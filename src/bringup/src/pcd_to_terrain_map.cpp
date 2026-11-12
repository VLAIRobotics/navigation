#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unordered_map>

#include <Eigen/Eigenvalues>
#include <fastdem/elevation_map.hpp>
#include <fastdem/io/npz.hpp>
#include <fastdem/postprocess/feature_extraction.hpp>
#include <fastdem/postprocess/inpainting.hpp>
#include <pcl/io/lzf.h>

#include "bringup/terrain_map_utils.hpp"

namespace
{
struct Options
{
  std::filesystem::path pcd_file;
  std::filesystem::path terrain_map_file;
  std::filesystem::path map_yaml_file;
  double resolution{0.10};
  double required_clearance_m{1.30};
  double ground_quantile{0.20};
  double max_ground_step{0.50};
  double analysis_radius{0.30};
  int min_ground_neighbors{1};
  std::size_t min_ground_component_cells{10};
  double ground_component_height_band{0.35};
  double max_ground_normal_angle{35.0};
  int min_plane_points{6};
  int min_valid_neighbors{16};
  int min_observations{3};
  double occupied_radius{0.0};
  std::filesystem::path display_pcd_file;
  double display_voxel_size{0.10};
  std::filesystem::path relocalization_pcd_file;
  double relocalization_voxel_size{0.20};
  std::uint32_t min_obstacle_points{20};
  double min_obstacle_fraction{0.05};
  std::size_t max_cells{25'000'000};
  std::string frame_id{"map"};
};

struct PcdHeader
{
  std::vector<std::string> fields;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<int> counts;
  std::size_t width{0};
  std::size_t height{0};
  std::size_t points{0};
  std::streamoff data_offset{0};
  std::string data;
};

struct BinaryCompressedPayload
{
  std::vector<char> uncompressed;
  std::vector<std::size_t> field_offsets;
};

struct CellStats
{
  struct HeightBin
  {
    int index{0};
    std::uint32_t count{0};
    float min_z{std::numeric_limits<float>::infinity()};
  };

  std::uint64_t count{0};
  double mean{0.0};
  double m2{0.0};
  float min_z{std::numeric_limits<float>::infinity()};
  float max_z{-std::numeric_limits<float>::infinity()};
  std::vector<float> samples;
  std::vector<HeightBin> height_bins;
  std::uint64_t random_state{0x9e3779b97f4a7c15ull};

  void add(float, float, float z)
  {
    ++count;
    const double delta = z - mean;
    mean += delta / static_cast<double>(count);
    m2 += delta * (z - mean);
    min_z = std::min(min_z, z);
    max_z = std::max(max_z, z);
    constexpr float kHeightBinSize = 0.10F;
    const int bin_index = static_cast<int>(std::floor(z / kHeightBinSize));
    const auto bin = std::find_if(
      height_bins.begin(), height_bins.end(), [bin_index](const HeightBin & value) {
        return value.index == bin_index;
      });
    if (bin == height_bins.end()) {
      height_bins.push_back({bin_index, 1, z});
    } else {
      ++bin->count;
      bin->min_z = std::min(bin->min_z, z);
    }
    constexpr std::size_t kReservoirSize = 64;
    if (samples.size() < kReservoirSize) {
      samples.push_back(z);
      return;
    }
    // A deterministic reservoir keeps the converter reproducible while bounding memory.
    random_state = random_state * 2862933555777941757ull + 3037000493ull;
    const std::uint64_t slot = random_state % count;
    if (slot < kReservoirSize) {
      const auto sample_index = static_cast<std::size_t>(slot);
      samples[sample_index] = z;
    }
  }

  float quantile(double q) const
  {
    if (samples.empty()) return std::numeric_limits<float>::quiet_NaN();
    auto values = samples;
    std::sort(values.begin(), values.end());
    const auto index = static_cast<std::size_t>(
      std::clamp(q, 0.0, 1.0) * static_cast<double>(values.size() - 1));
    return values[index];
  }

  float lowestDenseClusterQuantile(double q) const
  {
    if (height_bins.empty()) return std::numeric_limits<float>::quiet_NaN();
    auto bins = height_bins;
    std::sort(bins.begin(), bins.end(), [](const HeightBin & lhs, const HeightBin & rhs) {
      return lhs.index < rhs.index;
    });
    const auto dense_count = std::max<std::uint64_t>(
      3, static_cast<std::uint64_t>(std::ceil(static_cast<double>(count) * 0.05)));
    std::size_t first_begin = bins.size();
    std::size_t first_end = bins.size();
    std::uint64_t first_cluster_count = 0;
    std::size_t largest_begin = 0;
    std::size_t largest_end = 1;
    std::uint64_t largest_count = 0;
    for (std::size_t begin = 0; begin < bins.size();) {
      std::size_t end = begin + 1;
      std::uint64_t cluster_count = bins[begin].count;
      while (end < bins.size() && bins[end].index <= bins[end - 1].index + 1) {
        cluster_count += bins[end].count;
        ++end;
      }
      if (cluster_count >= dense_count && first_begin == bins.size()) {
        first_begin = begin; first_end = end; first_cluster_count = cluster_count;
      }
      if (cluster_count > largest_count) {
        largest_begin = begin; largest_end = end; largest_count = cluster_count;
      }
      begin = end;
    }
    if (first_begin == bins.size()) return quantile(q);
    // A tiny low-return cluster is usually a single grazing/noise surface.
    // Prefer the locally dominant height cluster in that case; the subsequent
    // neighbourhood plane check still rejects vertical wall patches.
    const bool low_cluster_supported =
      static_cast<double>(first_cluster_count) >= static_cast<double>(count) * 0.20;
    const auto begin = low_cluster_supported ? first_begin : largest_begin;
    const auto end = low_cluster_supported ? first_end : largest_end;
    std::vector<float> cluster_samples;
    for (const auto value : samples) {
      const auto bin_index = static_cast<int>(std::floor(value / 0.10F));
      if (bin_index >= bins[begin].index && bin_index <= bins[end - 1].index) {
        cluster_samples.push_back(value);
      }
    }
    if (!cluster_samples.empty()) {
      std::sort(cluster_samples.begin(), cluster_samples.end());
      const auto index = static_cast<std::size_t>(
        std::clamp(q, 0.0, 1.0) * static_cast<double>(cluster_samples.size() - 1));
      return cluster_samples[index];
    }
    return bins[begin].min_z;
  }

  float variance() const
  {
    return count < 2 ? 0.0f : static_cast<float>(m2 / static_cast<double>(count - 1));
  }
};

struct Bounds
{
  float min_x{std::numeric_limits<float>::infinity()};
  float min_y{std::numeric_limits<float>::infinity()};
  float max_x{-std::numeric_limits<float>::infinity()};
  float max_y{-std::numeric_limits<float>::infinity()};
  std::size_t finite_points{0};
};

struct DisplayVoxelKey
{
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};
  bool operator==(const DisplayVoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct DisplayVoxelHash
{
  std::size_t operator()(const DisplayVoxelKey & key) const
  {
    std::size_t hash = std::hash<std::int64_t>{}(key.x);
    hash ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    return hash;
  }
};

struct VoxelAccumulator
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  std::uint64_t count{0};

  void add(const float px, const float py, const float pz)
  {
    x += px;
    y += py;
    z += pz;
    ++count;
  }
};

void writeDisplayPcd(
  const std::filesystem::path & path,
  const std::unordered_map<DisplayVoxelKey, std::array<float, 3>, DisplayVoxelHash> & voxels)
{
  if (path.empty()) return;
  if (voxels.empty()) throw std::runtime_error("display PCD has no finite points");
  if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
  std::vector<std::pair<DisplayVoxelKey, std::array<float, 3>>> ordered(voxels.begin(), voxels.end());
  std::sort(ordered.begin(), ordered.end(), [](const auto & lhs, const auto & rhs) {
    if (lhs.first.x != rhs.first.x) return lhs.first.x < rhs.first.x;
    if (lhs.first.y != rhs.first.y) return lhs.first.y < rhs.first.y;
    return lhs.first.z < rhs.first.z;
  });
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("Cannot write display PCD: " + path.string());
  output << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
         << "WIDTH " << ordered.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << ordered.size() << "\nDATA binary\n";
  for (const auto & item : ordered) {
    output.write(reinterpret_cast<const char *>(item.second.data()), 3 * sizeof(float));
  }
  if (!output) throw std::runtime_error("Failed writing display PCD: " + path.string());
}

void writeRelocalizationPcd(
  const std::filesystem::path & path,
  const std::unordered_map<DisplayVoxelKey, VoxelAccumulator, DisplayVoxelHash> & voxels)
{
  if (path.empty()) return;
  if (voxels.empty()) throw std::runtime_error("relocalization PCD has no finite points");
  if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
  std::vector<DisplayVoxelKey> ordered;
  ordered.reserve(voxels.size());
  for (const auto & item : voxels) ordered.push_back(item.first);
  std::sort(ordered.begin(), ordered.end(), [](const auto & lhs, const auto & rhs) {
    if (lhs.x != rhs.x) return lhs.x < rhs.x;
    if (lhs.y != rhs.y) return lhs.y < rhs.y;
    return lhs.z < rhs.z;
  });
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("Cannot write relocalization PCD: " + path.string());
  output << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
         << "WIDTH " << ordered.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << ordered.size() << "\nDATA binary\n";
  for (const auto & key : ordered) {
    const auto & voxel = voxels.at(key);
    const float point[3]{
      static_cast<float>(voxel.x / voxel.count),
      static_cast<float>(voxel.y / voxel.count),
      static_cast<float>(voxel.z / voxel.count)};
    output.write(reinterpret_cast<const char *>(point), sizeof(point));
  }
  if (!output) throw std::runtime_error("Failed writing relocalization PCD: " + path.string());
}

bool hasSupportedGroundPlane(
  const std::vector<float> & ground, const std::vector<bool> & candidate,
  std::size_t rows, std::size_t cols, std::size_t row, std::size_t col,
  double resolution, double max_angle_degrees, std::size_t min_points)
{
  std::vector<Eigen::Vector3d> points;
  constexpr int kRadius = 2;
  for (int di = -kRadius; di <= kRadius; ++di) {
    for (int dj = -kRadius; dj <= kRadius; ++dj) {
      const auto ni = static_cast<long long>(row) + di;
      const auto nj = static_cast<long long>(col) + dj;
      if (ni < 0 || nj < 0 || ni >= static_cast<long long>(rows) || nj >= static_cast<long long>(cols)) continue;
      const auto flat = static_cast<std::size_t>(ni) * cols + static_cast<std::size_t>(nj);
      if (!candidate[flat] || !std::isfinite(ground[flat])) continue;
      points.emplace_back(di * resolution, dj * resolution, ground[flat]);
    }
  }
  // Sparse scan lines do not have enough 2-D support to make a reliable
  // normal decision; retain them and let connectivity/height evidence decide.
  if (points.size() < min_points) return true;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & point : points) centroid += point;
  centroid /= static_cast<double>(points.size());
  Eigen::Matrix2d xy = Eigen::Matrix2d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto & point : points) {
    const auto delta = point - centroid;
    xy(0, 0) += delta.x() * delta.x();
    xy(0, 1) += delta.x() * delta.y();
    xy(1, 0) += delta.y() * delta.x();
    xy(1, 1) += delta.y() * delta.y();
    covariance.noalias() += delta * delta.transpose();
  }
  xy /= static_cast<double>(points.size());
  if (xy.determinant() < resolution * resolution * resolution * resolution * 0.01) return true;
  covariance /= static_cast<double>(points.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) return true;
  constexpr double kPi = 3.14159265358979323846;
  const auto normal = solver.eigenvectors().col(0);
  return std::abs(normal.z()) >= std::cos(max_angle_degrees * kPi / 180.0);
}

std::vector<std::string> split(const std::string & line)
{
  std::istringstream stream(line);
  std::vector<std::string> values;
  std::string value;
  while (stream >> value) values.push_back(value);
  return values;
}

PcdHeader readHeader(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot open PCD: " + path.string());

  PcdHeader header;
  std::string line;
  while (std::getline(stream, line)) {
    const auto values = split(line);
    if (values.empty() || values[0][0] == '#') continue;
    const auto & key = values[0];
    if (key == "FIELDS") {
      header.fields.assign(values.begin() + 1, values.end());
    } else if (key == "SIZE") {
      for (std::size_t i = 1; i < values.size(); ++i) header.sizes.push_back(std::stoi(values[i]));
    } else if (key == "TYPE") {
      for (std::size_t i = 1; i < values.size(); ++i) header.types.push_back(values[i][0]);
    } else if (key == "COUNT") {
      for (std::size_t i = 1; i < values.size(); ++i) header.counts.push_back(std::stoi(values[i]));
    } else if (key == "WIDTH") {
      header.width = static_cast<std::size_t>(std::stoull(values.at(1)));
    } else if (key == "HEIGHT") {
      header.height = static_cast<std::size_t>(std::stoull(values.at(1)));
    } else if (key == "POINTS") {
      header.points = static_cast<std::size_t>(std::stoull(values.at(1)));
    } else if (key == "DATA") {
      header.data = values.at(1);
      header.data_offset = stream.tellg();
      break;
    }
  }
  if (header.counts.empty()) header.counts.assign(header.fields.size(), 1);
  if (header.fields.empty() || header.fields.size() != header.sizes.size() ||
    header.fields.size() != header.types.size() || header.fields.size() != header.counts.size() ||
    header.width == 0 || header.height == 0 || header.points == 0 || header.data.empty())
  {
    throw std::runtime_error("Incomplete PCD header: " + path.string());
  }
  if (header.width > std::numeric_limits<std::size_t>::max() / header.height) {
    throw std::runtime_error("PCD WIDTH*HEIGHT overflows size_t");
  }
  const auto dimension_points = header.width * header.height;
  if (dimension_points != header.points) {
    throw std::runtime_error(
      "PCD WIDTH*HEIGHT (" + std::to_string(header.width) + "*" +
      std::to_string(header.height) + "=" + std::to_string(dimension_points) +
      ") does not match POINTS (" + std::to_string(header.points) + ")");
  }
  if (header.data != "ascii" && header.data != "binary" && header.data != "binary_compressed") {
    throw std::runtime_error("Only ASCII, binary and binary_compressed PCD are supported: " + header.data);
  }
  for (const auto count : header.counts) {
    if (count != 1) throw std::runtime_error("PCD COUNT > 1 is not supported");
  }
  return header;
}

double readScalar(const char * data, char type, int size)
{
  if (type == 'F' && size == 4) {
    float value;
    std::memcpy(&value, data, sizeof(value));
    return value;
  }
  if (type == 'F' && size == 8) {
    double value;
    std::memcpy(&value, data, sizeof(value));
    return value;
  }
  throw std::runtime_error("PCD x/y/z fields must be float32 or float64");
}

BinaryCompressedPayload loadBinaryCompressedPayload(
  const std::filesystem::path & path, const PcdHeader & header)
{
  std::ifstream stream(path, std::ios::binary);
  stream.seekg(header.data_offset);
  if (!stream) throw std::runtime_error("Cannot seek PCD data: " + path.string());

  std::uint32_t compressed_size = 0;
  std::uint32_t uncompressed_size = 0;
  stream.read(reinterpret_cast<char *>(&compressed_size), sizeof(compressed_size));
  stream.read(reinterpret_cast<char *>(&uncompressed_size), sizeof(uncompressed_size));
  if (!stream || compressed_size == 0 || uncompressed_size == 0) {
    throw std::runtime_error("Invalid binary_compressed PCD size header");
  }
  std::vector<char> compressed(compressed_size);
  std::vector<char> uncompressed(uncompressed_size);
  stream.read(compressed.data(), static_cast<std::streamsize>(compressed.size()));
  if (!stream) throw std::runtime_error("Truncated binary_compressed PCD payload");
  const auto decoded = pcl::lzfDecompress(
    compressed.data(), compressed_size, uncompressed.data(), uncompressed_size);
  if (decoded != uncompressed_size) {
    throw std::runtime_error("Failed to decompress binary_compressed PCD payload");
  }

  BinaryCompressedPayload payload;
  payload.uncompressed = std::move(uncompressed);
  payload.field_offsets.resize(header.fields.size());
  std::size_t field_bytes = 0;
  for (std::size_t i = 0; i < header.fields.size(); ++i) {
    if (header.sizes[i] <= 0 || header.points >
      std::numeric_limits<std::size_t>::max() / static_cast<std::size_t>(header.sizes[i]))
    {
      throw std::runtime_error("Invalid binary_compressed PCD field size");
    }
    payload.field_offsets[i] = field_bytes;
    field_bytes += static_cast<std::size_t>(header.sizes[i]) * header.points;
  }
  if (field_bytes != payload.uncompressed.size()) {
    throw std::runtime_error(
      "binary_compressed PCD payload size does not match WIDTH*HEIGHT field layout");
  }
  return payload;
}

template<typename Callback>
void forEachPoint(
  const std::filesystem::path & path, const PcdHeader & header,
  const BinaryCompressedPayload * compressed_payload, Callback callback)
{
  const auto x_index = std::find(header.fields.begin(), header.fields.end(), "x");
  const auto y_index = std::find(header.fields.begin(), header.fields.end(), "y");
  const auto z_index = std::find(header.fields.begin(), header.fields.end(), "z");
  if (x_index == header.fields.end() || y_index == header.fields.end() || z_index == header.fields.end()) {
    throw std::runtime_error("PCD must contain x, y and z fields");
  }
  const auto x = static_cast<std::size_t>(x_index - header.fields.begin());
  const auto y = static_cast<std::size_t>(y_index - header.fields.begin());
  const auto z = static_cast<std::size_t>(z_index - header.fields.begin());
  std::vector<std::size_t> offsets(header.fields.size());
  std::size_t point_step = 0;
  for (std::size_t i = 0; i < header.fields.size(); ++i) {
    offsets[i] = point_step;
    point_step += static_cast<std::size_t>(header.sizes[i]);
  }

  if (header.data == "binary_compressed") {
    if (compressed_payload == nullptr) {
      throw std::runtime_error("binary_compressed PCD payload was not decoded");
    }
    for (std::size_t i = 0; i < header.points; ++i) {
      callback(
        static_cast<float>(readScalar(
          compressed_payload->uncompressed.data() + compressed_payload->field_offsets[x] +
          i * header.sizes[x], header.types[x], header.sizes[x])),
        static_cast<float>(readScalar(
          compressed_payload->uncompressed.data() + compressed_payload->field_offsets[y] +
          i * header.sizes[y], header.types[y], header.sizes[y])),
        static_cast<float>(readScalar(
          compressed_payload->uncompressed.data() + compressed_payload->field_offsets[z] +
          i * header.sizes[z], header.types[z], header.sizes[z])));
    }
    return;
  }

  std::ifstream stream(path, std::ios::binary);
  stream.seekg(header.data_offset);
  if (!stream) throw std::runtime_error("Cannot seek PCD data: " + path.string());

  if (header.data == "binary") {
    std::vector<char> point(point_step);
    for (std::size_t i = 0; i < header.points; ++i) {
      stream.read(point.data(), static_cast<std::streamsize>(point.size()));
      if (!stream) throw std::runtime_error("Truncated binary PCD payload");
      callback(
        static_cast<float>(readScalar(point.data() + offsets[x], header.types[x], header.sizes[x])),
        static_cast<float>(readScalar(point.data() + offsets[y], header.types[y], header.sizes[y])),
        static_cast<float>(readScalar(point.data() + offsets[z], header.types[z], header.sizes[z])));
    }
    return;
  }

  std::string line;
  for (std::size_t i = 0; i < header.points; ++i) {
    if (!std::getline(stream, line)) throw std::runtime_error("Truncated ASCII PCD payload");
    const auto values = split(line);
    if (values.size() != header.fields.size()) throw std::runtime_error("Invalid ASCII PCD row");
    callback(std::stof(values[x]), std::stof(values[y]), std::stof(values[z]));
  }
}

double optionValue(const std::string & argument, const std::string & name)
{
  const auto prefix = "--" + name + "=";
  if (argument.rfind(prefix, 0) != 0) throw std::runtime_error("Expected " + prefix + "<value>");
  return std::stod(argument.substr(prefix.size()));
}

std::string stringOption(const std::string & argument, const std::string & name)
{
  const auto prefix = "--" + name + "=";
  if (argument.rfind(prefix, 0) != 0) throw std::runtime_error("Expected " + prefix + "<value>");
  return argument.substr(prefix.size());
}

Options parseOptions(int argc, char ** argv)
{
  if (argc < 4) {
    throw std::runtime_error(
      "Usage: pcd_to_terrain_map <input.pcd> <terrain_map.npz> <map.yaml> "
      "[--resolution=0.10] [--ground-quantile=0.20] [--max-ground-step=0.50] "
      "[--required-clearance-m=1.30] [--analysis-radius=0.30] [--min-ground-neighbors=1] [--min-ground-component-cells=10] "
      "[--ground-component-height-band=0.35] "
      "[--max-ground-normal-angle=35] [--min-plane-points=6] "
      "[--min-valid-neighbors=16] "
      "[--occupied-radius=0.0] [--display-pcd=] [--display-voxel-size=0.10] "
      "[--relocalization-pcd=] [--relocalization-voxel-size=0.20] "
      "[--min-obstacle-points=20] [--min-obstacle-fraction=0.05] "
      "[--max-cells=25000000] [--frame=map]");
  }
  Options options;
  options.pcd_file = argv[1];
  options.terrain_map_file = argv[2];
  options.map_yaml_file = argv[3];
  for (int i = 4; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument.rfind("--resolution=", 0) == 0) options.resolution = optionValue(argument, "resolution");
    else if (argument.rfind("--required-clearance-m=", 0) == 0) options.required_clearance_m = optionValue(argument, "required-clearance-m");
    else if (argument.rfind("--ground-quantile=", 0) == 0) options.ground_quantile = optionValue(argument, "ground-quantile");
    else if (argument.rfind("--max-ground-step=", 0) == 0) options.max_ground_step = optionValue(argument, "max-ground-step");
    else if (argument.rfind("--analysis-radius=", 0) == 0) options.analysis_radius = optionValue(argument, "analysis-radius");
    else if (argument.rfind("--min-ground-neighbors=", 0) == 0) options.min_ground_neighbors = std::stoi(stringOption(argument, "min-ground-neighbors"));
    else if (argument.rfind("--min-ground-component-cells=", 0) == 0) options.min_ground_component_cells = static_cast<std::size_t>(std::stoull(stringOption(argument, "min-ground-component-cells")));
    else if (argument.rfind("--ground-component-height-band=", 0) == 0) options.ground_component_height_band = optionValue(argument, "ground-component-height-band");
    else if (argument.rfind("--max-ground-normal-angle=", 0) == 0) options.max_ground_normal_angle = optionValue(argument, "max-ground-normal-angle");
    else if (argument.rfind("--min-plane-points=", 0) == 0) options.min_plane_points = std::stoi(stringOption(argument, "min-plane-points"));
    else if (argument.rfind("--min-valid-neighbors=", 0) == 0) options.min_valid_neighbors = std::stoi(stringOption(argument, "min-valid-neighbors"));
    else if (argument.rfind("--min-observations=", 0) == 0) options.min_observations = std::stoi(stringOption(argument, "min-observations"));
    else if (argument.rfind("--occupied-radius=", 0) == 0) options.occupied_radius = optionValue(argument, "occupied-radius");
    else if (argument.rfind("--display-pcd=", 0) == 0) options.display_pcd_file = stringOption(argument, "display-pcd");
    else if (argument.rfind("--display-voxel-size=", 0) == 0) options.display_voxel_size = optionValue(argument, "display-voxel-size");
    else if (argument.rfind("--relocalization-pcd=", 0) == 0) options.relocalization_pcd_file = stringOption(argument, "relocalization-pcd");
    else if (argument.rfind("--relocalization-voxel-size=", 0) == 0) options.relocalization_voxel_size = optionValue(argument, "relocalization-voxel-size");
    else if (argument.rfind("--min-obstacle-points=", 0) == 0) options.min_obstacle_points = static_cast<std::uint32_t>(std::stoul(stringOption(argument, "min-obstacle-points")));
    else if (argument.rfind("--min-obstacle-fraction=", 0) == 0) options.min_obstacle_fraction = optionValue(argument, "min-obstacle-fraction");
    else if (argument.rfind("--max-cells=", 0) == 0) {
      options.max_cells = static_cast<std::size_t>(std::stoull(stringOption(argument, "max-cells")));
    }
    else if (argument.rfind("--frame=", 0) == 0) options.frame_id = stringOption(argument, "frame");
    else throw std::runtime_error("Unknown argument: " + argument);
  }
  if (!std::isfinite(options.resolution) || options.resolution <= 0.0 ||
    !std::isfinite(options.required_clearance_m) || options.required_clearance_m <= 0.0 ||
    options.required_clearance_m > std::numeric_limits<float>::max() ||
    !std::isfinite(options.ground_quantile) || options.ground_quantile < 0.0 || options.ground_quantile > 1.0 ||
    !std::isfinite(options.max_ground_step) || options.max_ground_step <= 0.0 ||
    !std::isfinite(options.analysis_radius) || options.analysis_radius <= 0.0 ||
    options.min_ground_neighbors < 0 || options.min_ground_component_cells == 0 ||
    !std::isfinite(options.ground_component_height_band) || options.ground_component_height_band < 0.0 ||
    !std::isfinite(options.max_ground_normal_angle) || options.max_ground_normal_angle <= 0.0 ||
    options.max_ground_normal_angle >= 90.0 || options.min_plane_points < 3 ||
    options.min_valid_neighbors < 3 || options.min_observations < 1 ||
    !std::isfinite(options.occupied_radius) || options.occupied_radius < 0.0 ||
    !std::isfinite(options.display_voxel_size) || options.display_voxel_size <= 0.0 ||
    !std::isfinite(options.relocalization_voxel_size) || options.relocalization_voxel_size <= 0.0 ||
    options.min_obstacle_points == 0 || !std::isfinite(options.min_obstacle_fraction) ||
    options.min_obstacle_fraction <= 0.0 || options.min_obstacle_fraction > 1.0 ||
    options.max_cells == 0 || options.frame_id.empty() ||
    options.occupied_radius / options.resolution >
    static_cast<double>(std::numeric_limits<int>::max() / 2))
  {
    throw std::runtime_error("Invalid terrain map generation option");
  }
  return options;
}

void writeYamlAndPgm(
  const std::filesystem::path & yaml_path, const fastdem::ElevationMap & map,
  const std::vector<std::uint8_t> & occupancy, double occupied_radius)
{
  const auto size = map.getSize();
  const auto rows = static_cast<std::size_t>(size(0));
  const auto cols = static_cast<std::size_t>(size(1));
  if (occupancy.size() != rows * cols) throw std::runtime_error("Occupancy dimensions do not match terrain map");
  auto pgm_path = yaml_path;
  pgm_path.replace_extension(".pgm");
  const auto pixels = bringup::occupancy_to_pgm(occupancy, rows, cols);
  std::ofstream pgm(pgm_path, std::ios::binary);
  if (!pgm) throw std::runtime_error("Cannot write PGM: " + pgm_path.string());
  // PGM width is world X and height is world Y.  GridMap uses the opposite
  // matrix order, so pixels already contain the required transpose/flip.
  pgm << "P5\n" << rows << " " << cols << "\n255\n";
  for (std::size_t row = 0; row < cols; ++row) {
    pgm.write(reinterpret_cast<const char *>(pixels.data() + row * rows),
      static_cast<std::streamsize>(rows));
  }
  if (!pgm) throw std::runtime_error("Failed writing PGM: " + pgm_path.string());

  const auto origin_x = map.getPosition().x() - map.getLength().x() / 2.0;
  const auto origin_y = map.getPosition().y() - map.getLength().y() / 2.0;
  std::ofstream yaml(yaml_path);
  if (!yaml) throw std::runtime_error("Cannot write YAML: " + yaml_path.string());
  yaml << std::setprecision(10)
       << "image: " << pgm_path.filename().string() << "\n"
       << "resolution: " << map.getResolution() << "\n"
       << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n"
       << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n";
  yaml << "# occupied_radius: " << occupied_radius << "\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parseOptions(argc, argv);
    const auto header = readHeader(options.pcd_file);
    std::optional<BinaryCompressedPayload> compressed_payload;
    if (header.data == "binary_compressed") {
      compressed_payload.emplace(loadBinaryCompressedPayload(options.pcd_file, header));
    }
    const auto * compressed_data = compressed_payload ? &*compressed_payload : nullptr;

    Bounds bounds;
    forEachPoint(options.pcd_file, header, compressed_data, [&bounds](float x, float y, float z) {
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return;
      bounds.min_x = std::min(bounds.min_x, x);
      bounds.min_y = std::min(bounds.min_y, y);
      bounds.max_x = std::max(bounds.max_x, x);
      bounds.max_y = std::max(bounds.max_y, y);
      ++bounds.finite_points;
    });
    if (bounds.finite_points == 0) throw std::runtime_error("PCD contains no finite XYZ points");

    const double length_x_double = static_cast<double>(bounds.max_x) - bounds.min_x + options.resolution;
    const double length_y_double = static_cast<double>(bounds.max_y) - bounds.min_y + options.resolution;
    bringup::checked_terrain_cell_count(
      length_x_double, length_y_double, options.resolution, options.max_cells);
    if (length_x_double > static_cast<double>(std::numeric_limits<float>::max()) ||
      length_y_double > static_cast<double>(std::numeric_limits<float>::max()))
    {
      throw std::runtime_error("terrain map dimensions exceed float range");
    }
    const float length_x = static_cast<float>(length_x_double);
    const float length_y = static_cast<float>(length_y_double);
    fastdem::ElevationMap map(length_x, length_y, static_cast<float>(options.resolution), options.frame_id);
    map.setPosition(nanogrid::Position(
      (bounds.min_x + bounds.max_x) / 2.0, (bounds.min_y + bounds.max_y) / 2.0));
    const auto feature_kernel = map.kernel(static_cast<float>(options.analysis_radius));
    const auto feature_size = map.getSize();
    const auto supported_kernel_size = [&](const int center_row, const int center_col) {
      return static_cast<int>(std::count_if(
        feature_kernel.entries.begin(), feature_kernel.entries.end(),
        [&](const auto & entry) {
          const int row = center_row + entry.dr;
          const int col = center_col + entry.dc;
          return row >= 0 && row < feature_size(0) && col >= 0 && col < feature_size(1);
        }));
    };
    int max_feature_neighbors = 0;
    for (const int center_row : {(feature_size(0) - 1) / 2, feature_size(0) / 2}) {
      for (const int center_col : {(feature_size(1) - 1) / 2, feature_size(1) / 2}) {
        max_feature_neighbors = std::max(
          max_feature_neighbors, supported_kernel_size(center_row, center_col));
      }
    }
    if (options.min_valid_neighbors > max_feature_neighbors) {
      std::ostringstream error;
      error << "min_valid_neighbors=" << options.min_valid_neighbors
            << " exceeds maximum in-map feature kernel capacity=" << max_feature_neighbors
            << " for resolution=" << options.resolution
            << " and analysis_radius=" << options.analysis_radius;
      throw std::invalid_argument(error.str());
    }
    for (const auto & layer : {
      std::string("elevation_min"), std::string("elevation_max"), std::string("variance"),
      std::string("n_points")}) {
      if (!map.exists(layer)) map.add(layer, NAN);
    }
    map.get("elevation").setConstant(NAN);
    map.get("elevation_min").setConstant(NAN);
    map.get("elevation_max").setConstant(NAN);
    map.get("variance").setConstant(NAN);
    map.get("n_points").setZero();
    map.get("observation_age").setZero();
    map.add("class", 0.0F);
    map.add("ground_source", 0.0F);
    map.add("clearance", std::numeric_limits<float>::infinity());
    map.get("class").setZero();
    map.get("ground_source").setZero();
    map.get("clearance").setConstant(std::numeric_limits<float>::infinity());

    const auto size = map.getSize();
    const auto rows = static_cast<std::size_t>(size(0));
    const auto cols = static_cast<std::size_t>(size(1));
    if (rows == 0 || cols == 0 || rows > options.max_cells / cols) {
      throw std::runtime_error("terrain map exceeds max_cells");
    }
    std::vector<CellStats> cells(rows * cols);
    std::unordered_map<DisplayVoxelKey, std::array<float, 3>, DisplayVoxelHash> display_voxels;
    if (!options.display_pcd_file.empty()) display_voxels.reserve(150'000);
    std::unordered_map<DisplayVoxelKey, VoxelAccumulator, DisplayVoxelHash> relocalization_voxels;
    if (!options.relocalization_pcd_file.empty()) relocalization_voxels.reserve(150'000);
    forEachPoint(options.pcd_file, header, compressed_data, [&map, &cells, &display_voxels, &relocalization_voxels, &options, cols](float x, float y, float z) {
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return;
      const auto index = map.index(nanogrid::Position(x, y));
      if (index) {
        cells[static_cast<std::size_t>(index->x()) * cols + static_cast<std::size_t>(index->y())].add(x, y, z);
      }
      if (!options.display_pcd_file.empty()) {
        const auto scale = options.display_voxel_size;
        const DisplayVoxelKey key{
          static_cast<std::int64_t>(std::floor(x / scale)),
          static_cast<std::int64_t>(std::floor(y / scale)),
          static_cast<std::int64_t>(std::floor(z / scale))};
        display_voxels.emplace(key, std::array<float, 3>{x, y, z});
      }
      if (!options.relocalization_pcd_file.empty()) {
        const auto scale = options.relocalization_voxel_size;
        const DisplayVoxelKey key{
          static_cast<std::int64_t>(std::floor(x / scale)),
          static_cast<std::int64_t>(std::floor(y / scale)),
          static_cast<std::int64_t>(std::floor(z / scale))};
        relocalization_voxels[key].add(x, y, z);
      }
    });
    writeDisplayPcd(options.display_pcd_file, display_voxels);
    writeRelocalizationPcd(options.relocalization_pcd_file, relocalization_voxels);

    std::vector<float> ground(rows * cols, NAN);
    std::vector<bool> candidate(rows * cols, false);
    std::size_t measured_cells = 0;
    std::size_t overhead_cells = 0;
    for (std::size_t i = 0; i < rows; ++i) {
      for (std::size_t j = 0; j < cols; ++j) {
        const auto flat = i * cols + j;
        const auto & stats = cells[flat];
        if (stats.count < static_cast<std::uint64_t>(options.min_observations)) continue;
        ++measured_cells;
        ground[flat] = stats.lowestDenseClusterQuantile(options.ground_quantile);
        candidate[flat] = std::isfinite(ground[flat]);
      }
    }

    std::size_t normal_rejected_cells = 0;
    for (std::size_t i = 0; i < rows; ++i) {
      for (std::size_t j = 0; j < cols; ++j) {
        const auto flat = i * cols + j;
        if (candidate[flat] && !hasSupportedGroundPlane(
            ground, candidate, rows, cols, i, j, options.resolution,
            options.max_ground_normal_angle,
            static_cast<std::size_t>(options.min_plane_points)))
        {
          candidate[flat] = false;
          ++normal_rejected_cells;
        }
      }
    }

    std::vector<bool> ground_valid = candidate;
    if (options.min_ground_neighbors > 0) {
      for (std::size_t i = 0; i < rows; ++i) {
        for (std::size_t j = 0; j < cols; ++j) {
          const auto flat = i * cols + j;
          if (!candidate[flat]) {
            ground_valid[flat] = false;
            continue;
          }
          int neighbors = 0;
          for (int di = -1; di <= 1; ++di) {
            for (int dj = -1; dj <= 1; ++dj) {
              if (di == 0 && dj == 0) continue;
              const auto ni = static_cast<long long>(i) + di;
              const auto nj = static_cast<long long>(j) + dj;
              if (ni < 0 || nj < 0 || ni >= static_cast<long long>(rows) || nj >= static_cast<long long>(cols)) continue;
              const auto nflat = static_cast<std::size_t>(ni) * cols + static_cast<std::size_t>(nj);
              if (candidate[nflat] && std::abs(ground[nflat] - ground[flat]) <= options.max_ground_step) ++neighbors;
            }
          }
          ground_valid[flat] = neighbors >= options.min_ground_neighbors;
        }
      }
    }
    const auto locally_valid_ground = ground_valid;
    ground_valid = bringup::retain_ground_components_by_area(
      ground, locally_valid_ground, rows, cols, options.max_ground_step,
      options.min_ground_component_cells, options.ground_component_height_band);
    std::size_t disconnected_surface_cells = 0;
    for (std::size_t i = 0; i < ground_valid.size(); ++i) {
      if (locally_valid_ground[i] && !ground_valid[i]) ++disconnected_surface_cells;
    }
    if (measured_cells > 0 && disconnected_surface_cells * 4 > measured_cells) {
      std::cerr << "pcd_to_terrain_map: warning: disconnected surface cells="
                << disconnected_surface_cells << " of " << measured_cells
                << " exceeds 25%; review the ground component selection\n";
    }

    // Restore narrow scan gaps and sparse boundaries from adjacent retained
    // ground seeds.  Expansion is deliberately local and height-continuous;
    // a wall-sized jump cannot seed traversable ground.
    std::size_t expanded_ground_cells = 0;
    for (std::size_t i = 0; i < rows; ++i) {
      for (std::size_t j = 0; j < cols; ++j) {
        const auto flat = i * cols + j;
        if (ground_valid[flat] || cells[flat].count == 0) continue;
        std::vector<float> neighbors;
        for (int di = -1; di <= 1; ++di) {
          for (int dj = -1; dj <= 1; ++dj) {
            if (di == 0 && dj == 0) continue;
            const auto ni = static_cast<long long>(i) + di;
            const auto nj = static_cast<long long>(j) + dj;
            if (ni < 0 || nj < 0 || ni >= static_cast<long long>(rows) || nj >= static_cast<long long>(cols)) continue;
            const auto nflat = static_cast<std::size_t>(ni) * cols + static_cast<std::size_t>(nj);
            if (ground_valid[nflat] && std::isfinite(ground[nflat])) neighbors.push_back(ground[nflat]);
          }
        }
        if (neighbors.size() < 2) continue;
        const auto [min_it, max_it] = std::minmax_element(neighbors.begin(), neighbors.end());
        if (*max_it - *min_it > options.max_ground_step) continue;
        std::sort(neighbors.begin(), neighbors.end());
        if (!std::isfinite(ground[flat])) ground[flat] = neighbors[neighbors.size() / 2];
        if (std::isfinite(ground[flat]) &&
          std::abs(ground[flat] - neighbors[neighbors.size() / 2]) > options.max_ground_step) continue;
        ground_valid[flat] = true;
        ++expanded_ground_cells;
      }
    }

    std::vector<std::uint8_t> occupancy(rows * cols, 205);
    std::vector<bool> occupied(rows * cols, false);
    std::size_t neighbor_ground_cells = 0;
    std::size_t blocking_cells = 0;
    std::size_t unknown_cells_without_evidence = 0;
    for (std::size_t i = 0; i < rows; ++i) {
      for (std::size_t j = 0; j < cols; ++j) {
        const auto flat = i * cols + j;
        const auto & stats = cells[flat];
        if (stats.count == 0) continue;
        const bool has_valid_ground = ground_valid[flat] && std::isfinite(ground[flat]);
        float estimated_ground = has_valid_ground ? ground[flat] : NAN;
        if (!std::isfinite(estimated_ground)) {
          // A wall/column cell often has no trustworthy local ground return.
          // Borrow only a nearby retained ground estimate for obstacle evidence;
          // do not turn the cell into traversable unknown merely because its
          // own ground plane was rejected.
          float best_distance = std::numeric_limits<float>::infinity();
          for (int di = -3; di <= 3; ++di) {
            for (int dj = -3; dj <= 3; ++dj) {
              const auto ni = static_cast<long long>(i) + di;
              const auto nj = static_cast<long long>(j) + dj;
              if (ni < 0 || nj < 0 || ni >= static_cast<long long>(rows) || nj >= static_cast<long long>(cols)) continue;
              const auto nflat = static_cast<std::size_t>(ni) * cols + static_cast<std::size_t>(nj);
              if (!ground_valid[nflat] || !std::isfinite(ground[nflat])) continue;
              const float distance = static_cast<float>(di * di + dj * dj);
              if (distance < best_distance) {
                best_distance = distance;
                estimated_ground = ground[nflat];
              }
            }
          }
          if (std::isfinite(estimated_ground)) ++neighbor_ground_cells;
        }
        std::vector<bringup::HeightEvidenceBin> evidence_bins;
        evidence_bins.reserve(stats.height_bins.size());
        for (const auto & bin : stats.height_bins) {
          evidence_bins.push_back({bin.index, bin.count, bin.min_z});
        }
        const auto obstacle_point = bringup::supported_height_above(
          evidence_bins, stats.count, estimated_ground, 0.15F,
          options.min_obstacle_points, options.min_obstacle_fraction);
        const auto terrain_class = bringup::classify_terrain_cell(
          has_valid_ground, estimated_ground, obstacle_point, static_cast<float>(options.required_clearance_m));
        if (terrain_class == bringup::TerrainCellClass::Unknown) {
          map.get("class")(static_cast<int>(i), static_cast<int>(j)) = 0.0F;
          occupancy[flat] = bringup::terrain_cell_occupancy(terrain_class);
          ++unknown_cells_without_evidence;
          continue;
        }
        if (has_valid_ground) {
          map.get("elevation")(static_cast<int>(i), static_cast<int>(j)) = estimated_ground;
          map.get("elevation_min")(static_cast<int>(i), static_cast<int>(j)) = estimated_ground;
          map.get("elevation_max")(static_cast<int>(i), static_cast<int>(j)) = estimated_ground;
        }
        map.get("variance")(static_cast<int>(i), static_cast<int>(j)) = stats.variance();
        map.get("n_points")(static_cast<int>(i), static_cast<int>(j)) = static_cast<float>(stats.count);
        const auto clearance_point = obstacle_point;
        const auto clearance = has_valid_ground && std::isfinite(clearance_point) ?
          clearance_point - estimated_ground : std::numeric_limits<float>::infinity();
        if (std::isfinite(clearance)) ++overhead_cells;
        const bool solid_obstacle = terrain_class == bringup::TerrainCellClass::Obstacle;
        map.get("clearance")(static_cast<int>(i), static_cast<int>(j)) = clearance;
        map.get("class")(static_cast<int>(i), static_cast<int>(j)) = solid_obstacle ? 2.0F : 1.0F;
        map.get("ground_source")(static_cast<int>(i), static_cast<int>(j)) =
          solid_obstacle ? 0.0F : 1.0F;
        if (solid_obstacle) {
          occupied[flat] = true;
          ++blocking_cells;
        }
        occupancy[flat] = bringup::terrain_cell_occupancy(terrain_class);
      }
    }

    const int radius_cells = static_cast<int>(std::ceil(options.occupied_radius / options.resolution));
    bringup::expand_occupied_cells(occupancy, occupied, rows, cols, radius_cells);

    fastdem::applyInpainting(map, 5, 3, true);
    fastdem::applyFeatureExtraction(
      map, static_cast<float>(options.analysis_radius), options.min_valid_neighbors,
      0.10f, 0.90f, options.min_observations, 0, "class");
    if (!fastdem::io::saveNpz(options.terrain_map_file.string(), map)) {
      throw std::runtime_error("Failed to write terrain map: " + options.terrain_map_file.string());
    }
    writeYamlAndPgm(options.map_yaml_file, map, occupancy, options.occupied_radius);

    std::size_t free_cells = 0;
    std::size_t occupied_cells = 0;
    std::size_t unknown_cells = 0;
    for (const auto value : occupancy) {
      if (value == 254) ++free_cells;
      else if (value == 0) ++occupied_cells;
      else ++unknown_cells;
    }
    std::cout << "Wrote terrain map " << options.terrain_map_file << " and " << options.map_yaml_file
              << " (" << rows << "x" << cols << ", finite points=" << bounds.finite_points
              << ", measured cells=" << measured_cells << ", overhead cells=" << overhead_cells
              << ", normal-rejected cells=" << normal_rejected_cells
              << ", disconnected surface cells=" << disconnected_surface_cells
              << ", retained ground cells=" << std::count(ground_valid.begin(), ground_valid.end(), true)
              << ", expanded ground cells=" << expanded_ground_cells
              << ", neighbor-ground cells=" << neighbor_ground_cells
              << ", blocking cells=" << blocking_cells
              << ", unknown-without-evidence=" << unknown_cells_without_evidence
              << ", display points=" << display_voxels.size()
              << ", relocalization points=" << relocalization_voxels.size()
              << ", free/occupied/unknown=" << free_cells << "/" << occupied_cells << "/" << unknown_cells
              << ")\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "pcd_to_terrain_map: " << error.what() << '\n';
    return 1;
  }
}
