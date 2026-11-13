// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * npz_io.cpp
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fastdem/io/npz.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace fastdem {
namespace io {

namespace detail {

// ─── CRC32 ──────────────────────────────────────────────────────────────────

constexpr std::array<uint32_t, 256> buildCrc32Table() {
  std::array<uint32_t, 256> table{};
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t c = i;
    for (int j = 0; j < 8; ++j) {
      c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    }
    table[i] = c;
  }
  return table;
}

static constexpr auto crc32Table = buildCrc32Table();

uint32_t crc32(const void* data, size_t len) {
  const auto* buf = static_cast<const uint8_t*>(data);
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc = crc32Table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
  }
  return crc ^ 0xFFFFFFFFu;
}

// ─── Little-endian helpers ──────────────────────────────────────────────────

void writeLE16(std::ostream& os, uint16_t v) {
  os.put(static_cast<char>(v & 0xFF));
  os.put(static_cast<char>((v >> 8) & 0xFF));
}

void writeLE32(std::ostream& os, uint32_t v) {
  os.put(static_cast<char>(v & 0xFF));
  os.put(static_cast<char>((v >> 8) & 0xFF));
  os.put(static_cast<char>((v >> 16) & 0xFF));
  os.put(static_cast<char>((v >> 24) & 0xFF));
}

uint16_t readLE16(std::istream& is) {
  uint8_t b[2];
  is.read(reinterpret_cast<char*>(b), 2);
  return static_cast<uint16_t>(b[0]) | (static_cast<uint16_t>(b[1]) << 8);
}

uint32_t readLE32(std::istream& is) {
  uint8_t b[4];
  is.read(reinterpret_cast<char*>(b), 4);
  return static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
         (static_cast<uint32_t>(b[2]) << 16) |
         (static_cast<uint32_t>(b[3]) << 24);
}

// ─── ZIP STORE writer ───────────────────────────────────────────────────────

struct ZipEntry {
  std::string name;
  uint32_t crc;
  uint32_t size;
  uint32_t offset;  // offset of local file header
};

void writeLocalHeader(std::ostream& os, const ZipEntry& e) {
  writeLE32(os, 0x04034b50);                                 // signature
  writeLE16(os, 20);                                          // version needed
  writeLE16(os, 0);                                           // flags
  writeLE16(os, 0);                                           // compression: STORE
  writeLE16(os, 0);                                           // mod time
  writeLE16(os, 0);                                           // mod date
  writeLE32(os, e.crc);                                       // CRC-32
  writeLE32(os, e.size);                                      // compressed size
  writeLE32(os, e.size);                                      // uncompressed size
  writeLE16(os, static_cast<uint16_t>(e.name.size()));        // filename length
  writeLE16(os, 0);                                           // extra field length
  os.write(e.name.data(), e.name.size());                     // filename
}

void writeCentralHeader(std::ostream& os, const ZipEntry& e) {
  writeLE32(os, 0x02014b50);                                  // signature
  writeLE16(os, 20);                                          // version made by
  writeLE16(os, 20);                                          // version needed
  writeLE16(os, 0);                                           // flags
  writeLE16(os, 0);                                           // compression: STORE
  writeLE16(os, 0);                                           // mod time
  writeLE16(os, 0);                                           // mod date
  writeLE32(os, e.crc);                                       // CRC-32
  writeLE32(os, e.size);                                      // compressed size
  writeLE32(os, e.size);                                      // uncompressed size
  writeLE16(os, static_cast<uint16_t>(e.name.size()));        // filename length
  writeLE16(os, 0);                                           // extra field length
  writeLE16(os, 0);                                           // comment length
  writeLE16(os, 0);                                           // disk number
  writeLE16(os, 0);                                           // internal attrs
  writeLE32(os, 0);                                           // external attrs
  writeLE32(os, e.offset);                                    // local header offset
  os.write(e.name.data(), e.name.size());                     // filename
}

void writeEndOfCentralDir(std::ostream& os, uint16_t count,
                          uint32_t cd_size, uint32_t cd_offset) {
  writeLE32(os, 0x06054b50);   // signature
  writeLE16(os, 0);            // disk number
  writeLE16(os, 0);            // disk with central dir
  writeLE16(os, count);        // entries on this disk
  writeLE16(os, count);        // total entries
  writeLE32(os, cd_size);      // central dir size
  writeLE32(os, cd_offset);    // central dir offset
  writeLE16(os, 0);            // comment length
}

// ─── NumPy .npy format ──────────────────────────────────────────────────────

std::vector<char> buildNpyArray(const float* data, int rows, int cols) {
  // Eigen::MatrixXf is column-major = Fortran order
  std::ostringstream dict;
  dict << "{'descr': '<f4', 'fortran_order': True, 'shape': ("
       << rows << ", " << cols << "), }";
  std::string dict_str = dict.str();

  // Pad to 64-byte alignment
  size_t prefix_len = 10;  // 6 (magic) + 2 (version) + 2 (header_len)
  size_t padding = 64 - ((prefix_len + dict_str.size() + 1) % 64);
  if (padding == 64) padding = 0;
  dict_str.append(padding, ' ');
  dict_str.push_back('\n');

  uint16_t header_len = static_cast<uint16_t>(dict_str.size());
  size_t data_bytes = static_cast<size_t>(rows) * cols * sizeof(float);

  std::vector<char> buf;
  buf.reserve(prefix_len + header_len + data_bytes);

  // Magic + version
  const char magic[] = {'\x93', 'N', 'U', 'M', 'P', 'Y', '\x01', '\x00'};
  buf.insert(buf.end(), magic, magic + 8);

  // Header length (little-endian)
  buf.push_back(static_cast<char>(header_len & 0xFF));
  buf.push_back(static_cast<char>((header_len >> 8) & 0xFF));

  // Header dict + data
  buf.insert(buf.end(), dict_str.begin(), dict_str.end());
  const char* raw = reinterpret_cast<const char*>(data);
  buf.insert(buf.end(), raw, raw + data_bytes);

  return buf;
}

std::vector<char> buildNpyString(const std::string& str) {
  std::ostringstream dict;
  dict << "{'descr': '|S" << str.size()
       << "', 'fortran_order': False, 'shape': (), }";
  std::string dict_str = dict.str();

  size_t prefix_len = 10;
  size_t padding = 64 - ((prefix_len + dict_str.size() + 1) % 64);
  if (padding == 64) padding = 0;
  dict_str.append(padding, ' ');
  dict_str.push_back('\n');

  uint16_t header_len = static_cast<uint16_t>(dict_str.size());

  std::vector<char> buf;
  buf.reserve(prefix_len + header_len + str.size());

  const char magic[] = {'\x93', 'N', 'U', 'M', 'P', 'Y', '\x01', '\x00'};
  buf.insert(buf.end(), magic, magic + 8);
  buf.push_back(static_cast<char>(header_len & 0xFF));
  buf.push_back(static_cast<char>((header_len >> 8) & 0xFF));

  buf.insert(buf.end(), dict_str.begin(), dict_str.end());
  buf.insert(buf.end(), str.begin(), str.end());

  return buf;
}

std::string escapeJson(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    if (c == '"')
      out += "\\\"";
    else if (c == '\\')
      out += "\\\\";
    else
      out += c;
  }
  return out;
}

constexpr int kMetadataVersion = 1;

std::string buildMetadataJson(const ElevationMap& map) {
  auto pos = map.getPosition();
  auto size = map.getSize();
  auto start = map.getStartIndex();

  std::ostringstream json;
  json << std::setprecision(std::numeric_limits<double>::max_digits10)
       << "{\"version\": " << kMetadataVersion
       << ", \"resolution\": " << map.getResolution()
       << ", \"position\": [" << pos.x() << ", " << pos.y() << "]"
       << ", \"frame_id\": \"" << escapeJson(map.getFrameId()) << "\""
       << ", \"size\": [" << size(0) << ", " << size(1) << "]"
       << ", \"start_index\": [" << start(0) << ", " << start(1) << "]"
       << "}";
  return json.str();
}

// ─── Metadata JSON parser (minimal) ────────────────────────────────────────

// Extract a numeric value after "key": from JSON string.
bool jsonFloat(const std::string& json, const std::string& key, float& out) {
  auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return false;
  pos = json.find(':', pos);
  if (pos == std::string::npos) return false;
  try {
    out = std::stof(json.substr(pos + 1));
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

size_t jsonFieldColon(const std::string& json, const std::string& key) {
  auto pos = json.find("\"" + key + "\"");
  while (pos != std::string::npos) {
    auto after_key = json.find_first_not_of(" \t\r\n", pos + key.size() + 2);
    if (after_key != std::string::npos && json[after_key] == ':') {
      return after_key;
    }
    pos = json.find("\"" + key + "\"", pos + key.size() + 2);
  }
  return std::string::npos;
}

bool jsonDouble(const std::string& json, const std::string& key, double& out) {
  const auto colon = jsonFieldColon(json, key);
  if (colon == std::string::npos) return false;
  const auto pos = json.find_first_not_of(" \t\r\n", colon + 1);
  if (pos == std::string::npos) return false;
  try {
    size_t consumed = 0;
    out = std::stod(json.substr(pos), &consumed);
    if (consumed == 0) return false;
    const auto next = json.find_first_not_of(" \t\r\n", pos + consumed);
    return next == std::string::npos || json[next] == ',' || json[next] == '}';
  } catch (const std::exception&) {
    return false;
  }
}

// Extract two ints from "key": [a, b].
bool jsonIntPair(const std::string& json, const std::string& key, int& a,
                 int& b) {
  auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return false;
  pos = json.find('[', pos);
  if (pos == std::string::npos) return false;
  auto end = json.find(']', pos);
  if (end == std::string::npos) return false;
  std::string inner = json.substr(pos + 1, end - pos - 1);
  auto comma = inner.find(',');
  if (comma == std::string::npos) return false;
  try {
    a = std::stoi(inner.substr(0, comma));
    b = std::stoi(inner.substr(comma + 1));
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

// Extract two doubles from "key": [a, b].
bool jsonDoublePair(const std::string& json, const std::string& key, double& a,
                    double& b) {
  auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return false;
  pos = json.find('[', pos);
  if (pos == std::string::npos) return false;
  auto end = json.find(']', pos);
  if (end == std::string::npos) return false;
  std::string inner = json.substr(pos + 1, end - pos - 1);
  auto comma = inner.find(',');
  if (comma == std::string::npos) return false;
  try {
    a = std::stod(inner.substr(0, comma));
    b = std::stod(inner.substr(comma + 1));
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

// Extract a quoted string value after "key": "value".
bool jsonString(const std::string& json, const std::string& key,
                std::string& out) {
  auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return false;
  pos = json.find(':', pos);
  if (pos == std::string::npos) return false;
  auto q1 = json.find('"', pos + 1);
  if (q1 == std::string::npos) return false;
  auto q2 = json.find('"', q1 + 1);
  if (q2 == std::string::npos) return false;
  out = json.substr(q1 + 1, q2 - q1 - 1);
  return true;
}

// ─── .npy header parser ────────────────────────────────────────────────────

struct NpyInfo {
  int rows = 0;
  int cols = 0;
  bool is_float_array = false;  // '<f4' with shape (r,c)
  bool is_string = false;       // '|S...' with shape ()
  bool is_fortran_order = false;
  bool is_scalar_string = false;
  size_t string_len = 0;
  size_t data_offset = 0;  // offset to data within .npy blob
};

bool parseNpyHeader(const char* buf, size_t buf_size, NpyInfo& info) {
  // Validate magic: \x93NUMPY
  if (buf_size < 10) return false;
  if (buf[0] != '\x93' || buf[1] != 'N' || buf[2] != 'U' || buf[3] != 'M' ||
      buf[4] != 'P' || buf[5] != 'Y') {
    return false;
  }

  uint16_t header_len = static_cast<uint8_t>(buf[8]) |
                         (static_cast<uint16_t>(static_cast<uint8_t>(buf[9]))
                          << 8);
  info.data_offset = 10 + header_len;
  if (info.data_offset > buf_size) return false;

  std::string dict(buf + 10, header_len);

  // Check descr
  auto descr_pos = dict.find("'descr'");
  if (descr_pos == std::string::npos) return false;

  if (dict.find("'<f4'") != std::string::npos) {
    info.is_float_array = true;
    info.is_fortran_order = dict.find("'fortran_order': True") != std::string::npos;
    // Parse shape: (rows, cols)
    auto shape_pos = dict.find("'shape'");
    if (shape_pos == std::string::npos) return false;
    auto paren = dict.find('(', shape_pos);
    if (paren == std::string::npos) return false;
    auto paren_end = dict.find(')', paren);
    if (paren_end == std::string::npos) return false;
    std::string shape_str = dict.substr(paren + 1, paren_end - paren - 1);
    auto comma = shape_str.find(',');
    if (comma == std::string::npos) return false;
    try {
      info.rows = std::stoi(shape_str.substr(0, comma));
      std::string cols_str = shape_str.substr(comma + 1);
      // Handle trailing comma: "(200, 100, )"
      if (!cols_str.empty() && cols_str.back() == ',') cols_str.pop_back();
      while (!cols_str.empty() && cols_str.front() == ' ') cols_str.erase(0, 1);
      if (cols_str.empty()) return false;
      info.cols = std::stoi(cols_str);
    } catch (const std::exception&) {
      return false;
    }
  } else if (dict.find("'|S") != std::string::npos) {
    info.is_string = true;
    info.is_scalar_string = dict.find("'shape': ()") != std::string::npos;
    auto s_pos = dict.find("'|S");
    auto s_end = dict.find('\'', s_pos + 3);
    if (s_end == std::string::npos) return false;
    try {
      info.string_len = std::stoul(dict.substr(s_pos + 3, s_end - s_pos - 3));
    } catch (const std::exception&) {
      return false;
    }
  } else {
    return false;
  }

  return true;
}

}  // namespace detail

// ─── Save ───────────────────────────────────────────────────────────────────

bool saveNpz(const std::string& filename, const ElevationMap& map) {
  return saveNpz(filename, map, map.getLayers());
}

bool saveNpz(const std::string& filename, const ElevationMap& map,
             const std::vector<std::string>& layer_names) {
  std::ofstream fs(filename, std::ios::binary);
  if (!fs.is_open()) {
    spdlog::error("[npz_io] Cannot create {}", filename);
    return false;
  }

  std::vector<detail::ZipEntry> entries;
  int rows = map.getSize()(0);
  int cols = map.getSize()(1);

  // Helper: add a .npy blob to the archive
  auto addEntry = [&](const std::string& name, const std::vector<char>& npy) {
    detail::ZipEntry e;
    e.name = name;
    e.crc = detail::crc32(npy.data(), npy.size());
    e.size = static_cast<uint32_t>(npy.size());
    e.offset = static_cast<uint32_t>(fs.tellp());
    detail::writeLocalHeader(fs, e);
    fs.write(npy.data(), npy.size());
    entries.push_back(std::move(e));
  };

  // Write each layer
  for (const auto& name : layer_names) {
    if (!map.exists(name)) {
      spdlog::warn("[npz_io] Layer '{}' does not exist, skipping", name);
      continue;
    }
    const auto& matrix = map.get(name);
    addEntry(name + ".npy", detail::buildNpyArray(matrix.data(), rows, cols));
  }

  // Write metadata
  std::string meta_json = detail::buildMetadataJson(map);
  addEntry("meta.npy", detail::buildNpyString(meta_json));

  // Central directory
  uint32_t cd_offset = static_cast<uint32_t>(fs.tellp());
  for (const auto& e : entries) {
    detail::writeCentralHeader(fs, e);
  }
  uint32_t cd_size = static_cast<uint32_t>(fs.tellp()) - cd_offset;

  // End of central directory
  detail::writeEndOfCentralDir(fs, static_cast<uint16_t>(entries.size()),
                               cd_size, cd_offset);

  if (fs.fail()) {
    spdlog::error("[npz_io] Write failed for {}", filename);
    return false;
  }

  return true;
}

// ─── Load ───────────────────────────────────────────────────────────────────

bool loadNpz(const std::string& filename, ElevationMap& map) {
  return loadNpz(filename, map, nullptr);
}

bool loadNpz(const std::string& filename, ElevationMap& map,
             std::string* error) {
  if (error != nullptr) error->clear();
  const auto fail = [error](const std::string& message) {
    if (error != nullptr) *error = message;
    spdlog::error("[npz_io] {}", message);
    return false;
  };

  std::ifstream fs(filename, std::ios::binary);
  if (!fs.is_open()) {
    return fail("cannot open NPZ: " + filename);
  }

  // Collect entries by reading local file headers sequentially
  struct Entry {
    std::string name;
    std::vector<char> data;
  };
  std::vector<Entry> entries;
  std::string metadata_json;

  constexpr uint32_t kLocalSig = 0x04034b50;
  constexpr uint32_t kMaxEntries = 1000;
  constexpr uint32_t kMaxEntrySize = 400'000'000;  // 400MB
  constexpr size_t kMaxMapCells = kMaxEntrySize / sizeof(float);

  while (entries.size() < kMaxEntries) {
    uint32_t sig = detail::readLE32(fs);
    if (fs.fail() || sig != kLocalSig) break;

    fs.ignore(2);  // version needed
    const uint16_t flags = detail::readLE16(fs);
    const uint16_t compression = detail::readLE16(fs);
    fs.ignore(8);  // time, date, crc32
    const uint32_t compressed_size = detail::readLE32(fs);
    uint32_t uncompressed_size = detail::readLE32(fs);
    uint16_t name_len = detail::readLE16(fs);
    uint16_t extra_len = detail::readLE16(fs);
    if (fs.fail()) break;

    if (flags != 0 || compression != 0 || compressed_size != uncompressed_size) {
      return fail("NPZ entries must be unencrypted and stored without compression");
    }
    if (name_len == 0 || name_len > 4096 || uncompressed_size > kMaxEntrySize) {
      return fail("NPZ entry has an invalid name or exceeds the 400 MB limit");
    }

    std::string name(name_len, '\0');
    fs.read(&name[0], name_len);
    fs.ignore(extra_len);
    if (fs.fail()) return fail("truncated NPZ entry header");
    if (std::any_of(entries.begin(), entries.end(), [&name](const Entry& entry) {
        return entry.name == name;
      }))
    {
      return fail("NPZ contains duplicate entry: " + name);
    }

    std::vector<char> data(uncompressed_size);
    fs.read(data.data(), uncompressed_size);
    if (fs.fail()) {
      return fail("truncated data for NPZ entry: " + name);
    }

    entries.push_back({std::move(name), std::move(data)});
  }

  if (entries.empty()) {
    return fail("NPZ has no readable entries: " + filename);
  }

  // Find and parse metadata entry
  bool found_meta = false;
  for (const auto& e : entries) {
    if (e.name == "meta.npy") {
      detail::NpyInfo info;
      if (!detail::parseNpyHeader(e.data.data(), e.data.size(), info) ||
          !info.is_string || !info.is_scalar_string)
      {
        return fail("meta.npy must contain a scalar string");
      }
      if (info.string_len > e.data.size() - info.data_offset) {
        return fail("meta.npy string data is truncated");
      }
      metadata_json.assign(e.data.data() + info.data_offset, info.string_len);
      found_meta = true;
      break;
    }
  }

  if (!found_meta) {
    return fail("NPZ is missing meta.npy: " + filename);
  }

  // Parse metadata
  double version = 0;
  float resolution = 0;
  double pos_x = 0, pos_y = 0;
  int size_rows = 0, size_cols = 0;
  int start_x = 0, start_y = 0;
  std::string frame_id;

  // Version check (absent in pre-v1 files → allow gracefully)
  if (detail::jsonFieldColon(metadata_json, "version") != std::string::npos) {
    if (!detail::jsonDouble(metadata_json, "version", version) ||
      !std::isfinite(version) || version < 0.0 ||
      version > static_cast<double>(detail::kMetadataVersion) || std::trunc(version) != version)
    {
      return fail("NPZ metadata version is invalid or unsupported");
    }
  }

  if (!detail::jsonFloat(metadata_json, "resolution", resolution) ||
      !detail::jsonDoublePair(metadata_json, "position", pos_x, pos_y) ||
      !detail::jsonIntPair(metadata_json, "size", size_rows, size_cols)) {
    return fail("NPZ metadata is missing resolution, position, or size");
  }
  if (!detail::jsonString(metadata_json, "frame_id", frame_id) || frame_id.empty()) {
    return fail("NPZ metadata frame_id is missing or empty");
  }
  if (!detail::jsonIntPair(metadata_json, "start_index", start_x, start_y)) {
    return fail("NPZ metadata start_index is missing or malformed");
  }

  if (size_rows <= 0 || size_cols <= 0 ||
    static_cast<size_t>(size_rows) > kMaxMapCells / static_cast<size_t>(size_cols) ||
    !std::isfinite(resolution) || resolution <= 0.0F ||
    !std::isfinite(pos_x) || !std::isfinite(pos_y) ||
    start_x < 0 || start_y < 0 || start_x >= size_rows || start_y >= size_cols)
  {
    return fail("NPZ metadata has invalid dimensions, geometry, or start_index");
  }

  const float length_x = resolution * size_rows;
  const float length_y = resolution * size_cols;
  if (!std::isfinite(length_x) || !std::isfinite(length_y) ||
    length_x <= 0.0F || length_y <= 0.0F)
  {
    return fail("NPZ metadata produces invalid map lengths");
  }
  ElevationMap loaded_map;
  loaded_map.setFrameId(frame_id);
  loaded_map.setGeometry(length_x, length_y, resolution);
  loaded_map.setPosition(nanogrid::Position(pos_x, pos_y));
  loaded_map.setStartIndex(nanogrid::Index(start_x, start_y));

  const size_t expected_bytes =
    static_cast<size_t>(size_rows) * static_cast<size_t>(size_cols) * sizeof(float);
  int layers_loaded = 0;

  for (const auto& e : entries) {
    if (e.name == "meta.npy") continue;

    // Strip .npy suffix to get layer name
    if (e.name.size() <= 4 || e.name.substr(e.name.size() - 4) != ".npy") {
      continue;
    }
    std::string layer_name = e.name.substr(0, e.name.size() - 4);
    if (layer_name.empty()) return fail("NPZ contains an empty layer name");

    detail::NpyInfo info;
    if (!detail::parseNpyHeader(e.data.data(), e.data.size(), info) ||
      !info.is_float_array)
    {
      return fail("layer '" + layer_name + "' must be a two-dimensional float32 array");
    }

    if (info.rows != size_rows || info.cols != size_cols ||
      !info.is_fortran_order)
    {
      return fail("layer '" + layer_name +
        "' has an unexpected shape or is not stored in column-major order");
    }

    if (info.data_offset > e.data.size() ||
      expected_bytes != e.data.size() - info.data_offset)
    {
      return fail("layer '" + layer_name + "' has an invalid data length");
    }

    if (!loaded_map.exists(layer_name)) {
      loaded_map.add(layer_name);
    }
    auto& matrix = loaded_map.get(layer_name);
    if (matrix.rows() != size_rows || matrix.cols() != size_cols ||
      static_cast<size_t>(matrix.size()) != static_cast<size_t>(size_rows) * size_cols)
    {
      return fail("layer '" + layer_name + "' does not match map geometry");
    }
    std::memcpy(matrix.data(), e.data.data() + info.data_offset,
                expected_bytes);
    ++layers_loaded;
  }

  if (layers_loaded == 0 || !loaded_map.exists(layer::elevation)) {
    return fail("NPZ must contain at least one data layer including elevation");
  }

  map = std::move(loaded_map);
  return true;
}

}  // namespace io
}  // namespace fastdem
