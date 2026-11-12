#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/ann/traits.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using CovarianceCloud = pcl::PointCloud<pcl::PointCovariance>;
using CovarianceTree = small_gicp::KdTree<CovarianceCloud>;

struct Options {
  std::filesystem::path snapshot_dir;
  std::filesystem::path map_path;
  std::vector<int> threads{2, 4};
  std::vector<int> iterations{10, 20, 30};
  std::vector<double> translation_perturbations{0.0, 0.25, 0.50};
  std::vector<double> yaw_perturbations{0.0, 0.15, 0.30};
  double voxel_size{0.20};
  double max_correspondence_distance{1.0};
  std::filesystem::path output{"gicp_offline_benchmark.csv"};
};

template<typename T>
std::vector<T> parseList(const std::string & value)
{
  std::vector<T> result;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    if (!item.empty()) {
      if constexpr (std::is_same_v<T, int>) {
        result.push_back(std::stoi(item));
      } else {
        result.push_back(std::stod(item));
      }
    }
  }
  if (result.empty()) {
    throw std::invalid_argument("list option must not be empty: " + value);
  }
  return result;
}

std::string optionValue(int argc, char ** argv, const std::string & name, const std::string & fallback)
{
  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    const std::string prefix = name + "=";
    if (argument.rfind(prefix, 0) == 0) {
      return argument.substr(prefix.size());
    }
    if (argument == name && i + 1 < argc) {
      return argv[i + 1];
    }
  }
  return fallback;
}

Options parseOptions(int argc, char ** argv)
{
  Options options;
  const auto snapshot = optionValue(argc, argv, "--snapshot-dir", "");
  if (snapshot.empty()) {
    throw std::invalid_argument("--snapshot-dir is required");
  }
  options.snapshot_dir = snapshot;
  options.map_path = optionValue(argc, argv, "--map", "");
  options.threads = parseList<int>(optionValue(argc, argv, "--threads", "2,4"));
  options.iterations = parseList<int>(optionValue(argc, argv, "--iterations", "10,20,30"));
  options.translation_perturbations = parseList<double>(
    optionValue(argc, argv, "--translation-perturbations", "0,0.25,0.50"));
  options.yaw_perturbations = parseList<double>(
    optionValue(argc, argv, "--yaw-perturbations", "0,0.15,0.30"));
  options.voxel_size = std::stod(optionValue(argc, argv, "--voxel-size", "0.20"));
  options.max_correspondence_distance = std::stod(
    optionValue(argc, argv, "--max-correspondence-distance", "1.0"));
  options.output = optionValue(argc, argv, "--output", options.output.string());
  return options;
}

Eigen::Isometry3d readMatrix(const std::filesystem::path & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot open matrix file: " + path.string());
  }
  Eigen::Matrix4d matrix;
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      if (!(file >> matrix(row, column))) {
        throw std::runtime_error("matrix file must contain 16 values: " + path.string());
      }
    }
  }
  Eigen::Isometry3d result;
  result.matrix() = matrix;
  return result;
}

struct GeometryMetrics {
  double rmse_m{std::numeric_limits<double>::infinity()};
  double p95_m{std::numeric_limits<double>::infinity()};
};

GeometryMetrics geometryMetrics(
  const Eigen::Isometry3d & transform, const CovarianceCloud & source,
  const CovarianceTree & tree, double max_distance)
{
  std::vector<double> distances;
  distances.reserve(source.size());
  double squared_sum = 0.0;
  const double max_distance_sq = max_distance * max_distance;
  for (const auto & point : source) {
    const Eigen::Vector3d transformed = transform * Eigen::Vector3d(point.x, point.y, point.z);
    const Eigen::Vector4d query(transformed.x(), transformed.y(), transformed.z(), 1.0);
    std::size_t index = 0;
    double squared_distance = 0.0;
    if (transformed.allFinite() && small_gicp::traits::nearest_neighbor_search(
        tree, query, &index, &squared_distance) != 0 &&
      std::isfinite(squared_distance) && squared_distance <= max_distance_sq)
    {
      distances.push_back(std::sqrt(std::max(0.0, squared_distance)));
      squared_sum += squared_distance;
    }
  }
  if (!distances.empty()) {
    GeometryMetrics metrics;
    metrics.rmse_m = std::sqrt(squared_sum / static_cast<double>(distances.size()));
    const auto p95_index = static_cast<std::size_t>(0.95 * static_cast<double>(distances.size() - 1));
    std::nth_element(distances.begin(), distances.begin() + p95_index, distances.end());
    metrics.p95_m = distances[p95_index];
    return metrics;
  }
  return {};
}

struct RegistrationSample {
  small_gicp::RegistrationResult result;
  GeometryMetrics geometry;
  double elapsed_ms{0.0};
};

RegistrationSample registerOnce(
  const CovarianceCloud & map, const CovarianceCloud & source, const CovarianceTree & tree,
  const Eigen::Isometry3d & seed, int threads, int iterations, double max_distance)
{
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
  registration.reduction.num_threads = threads;
  registration.rejector.max_dist_sq = max_distance * max_distance;
  registration.optimizer.max_iterations = iterations;
  registration.criteria.rotation_eps = 0.1 * 3.14159265358979323846 / 180.0;
  registration.criteria.translation_eps = 1e-3;
  const auto start = std::chrono::steady_clock::now();
  RegistrationSample sample;
  sample.result = registration.align(map, source, tree, seed);
  sample.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  sample.geometry = geometryMetrics(sample.result.T_target_source, source, tree, max_distance);
  return sample;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parseOptions(argc, argv);
    const auto map_path = options.map_path.empty() ? options.snapshot_dir / "map.pcd" : options.map_path;
    const auto snapshot_map_path = options.snapshot_dir / "clean_map.pcd";
    const auto scan_path = options.snapshot_dir / "cloud_registered_body_downsampled.pcd";
    const auto seed_path = options.snapshot_dir / "seed_map_body.txt";
    const auto actual_map_path = std::filesystem::is_regular_file(map_path) ? map_path : snapshot_map_path;
    PointCloud raw_map;
    PointCloud raw_source;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(actual_map_path.string(), raw_map) != 0 || raw_map.empty()) {
      throw std::runtime_error("cannot load map PCD; copy the registration map to " + map_path.string());
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_path.string(), raw_source) != 0 || raw_source.empty()) {
      throw std::runtime_error("cannot load snapshot scan: " + scan_path.string());
    }
    const Eigen::Isometry3d base_seed = readMatrix(seed_path);
    std::ofstream csv(options.output);
    if (!csv) {
      throw std::runtime_error("cannot open output CSV: " + options.output.string());
    }
    csv << "threads,max_iterations,translation_perturbation_m,yaw_perturbation_rad,"
           "map_points,source_points,registration_ms,converged,inliers,rmse_m,p95_m\n";
    csv << std::fixed << std::setprecision(6);

    for (const int threads : options.threads) {
      if (threads < 1) {
        throw std::invalid_argument("thread counts must be positive");
      }
      auto map_cloud = small_gicp::voxelgrid_sampling_omp<PointCloud, CovarianceCloud>(
        raw_map, options.voxel_size, threads);
      if (map_cloud->size() < 20) {
        throw std::runtime_error("map has too few points after voxelization");
      }
      small_gicp::estimate_covariances_omp(*map_cloud, 20, threads);
      auto map_tree = std::make_shared<CovarianceTree>(
        map_cloud, small_gicp::KdTreeBuilderOMP(threads));
      auto source_cloud = small_gicp::voxelgrid_sampling_omp<PointCloud, CovarianceCloud>(
        raw_source, options.voxel_size, threads);
      small_gicp::estimate_covariances_omp(*source_cloud, 20, threads);

      for (const int iterations : options.iterations) {
        for (const double translation_perturbation : options.translation_perturbations) {
          for (const double yaw_perturbation : options.yaw_perturbations) {
            Eigen::Isometry3d seed = base_seed;
            seed.translation().x() += translation_perturbation;
            seed.linear() = Eigen::AngleAxisd(
              yaw_perturbation, Eigen::Vector3d::UnitZ()).toRotationMatrix() * seed.linear();
            const auto sample = registerOnce(
              *map_cloud, *source_cloud, *map_tree, seed, threads, iterations,
              options.max_correspondence_distance);
            csv << threads << ',' << iterations << ',' << translation_perturbation << ','
                << yaw_perturbation << ',' << map_cloud->size() << ',' << source_cloud->size() << ','
                << sample.elapsed_ms << ',' << (sample.result.converged ? 1 : 0) << ','
                << sample.result.num_inliers << ',' << sample.geometry.rmse_m << ','
                << sample.geometry.p95_m << '\n';
          }
        }
      }
    }
    std::cout << "Wrote offline GICP benchmark: " << options.output << '\n';
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "gicp_offline_benchmark: " << exception.what() << '\n';
    return 2;
  }
}
