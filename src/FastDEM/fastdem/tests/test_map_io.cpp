// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>

#include "fastdem/elevation_map.hpp"
#include "fastdem/io/npz.hpp"
#include "fastdem/io/png.hpp"

using namespace fastdem;

namespace {

bool replaceNpyHeaderText(std::string& archive, const std::string& entry_name,
                          const std::string& from, const std::string& to) {
  if (from.size() != to.size()) return false;
  const auto entry = archive.find(entry_name);
  if (entry == std::string::npos) return false;
  const auto npy = entry + entry_name.size();
  if (npy + 10 > archive.size()) return false;
  const auto header_len = static_cast<std::size_t>(
      static_cast<unsigned char>(archive[npy + 8])) |
    (static_cast<std::size_t>(static_cast<unsigned char>(archive[npy + 9])) << 8);
  const auto header = npy + 10;
  if (header > archive.size() || header_len > archive.size() - header) return false;
  const auto field = archive.find(from, header);
  if (field == std::string::npos || field + from.size() > header + header_len) return false;
  archive.replace(field, from.size(), to);
  return true;
}

bool replaceMetaText(std::string& archive, const std::string& from,
                     const std::string& to) {
  if (from.size() != to.size()) return false;
  const auto entry = archive.find("meta.npy");
  if (entry == std::string::npos) return false;
  const auto npy = entry + std::string("meta.npy").size();
  if (npy + 10 > archive.size()) return false;
  const auto header_len = static_cast<std::size_t>(
      static_cast<unsigned char>(archive[npy + 8])) |
    (static_cast<std::size_t>(static_cast<unsigned char>(archive[npy + 9])) << 8);
  const auto data = npy + 10 + header_len;
  if (data > archive.size()) return false;
  const auto field = archive.find(from, data);
  if (field == std::string::npos) return false;
  archive.replace(field, from.size(), to);
  return true;
}

bool replaceMetaTextKeepingSize(std::string& archive, const std::string& from,
                                const std::string& to) {
  const auto entry = archive.find("meta.npy");
  if (entry == std::string::npos) return false;
  const auto npy = entry + std::string("meta.npy").size();
  if (npy + 10 > archive.size()) return false;
  const auto header_len = static_cast<std::size_t>(
      static_cast<unsigned char>(archive[npy + 8])) |
    (static_cast<std::size_t>(static_cast<unsigned char>(archive[npy + 9])) << 8);
  const auto data = npy + 10 + header_len;
  if (data > archive.size()) return false;
  const auto json_end = archive.find('}', data);
  const auto field = archive.find(from, data);
  if (json_end == std::string::npos || field == std::string::npos || field > json_end ||
    from.size() > json_end + 1 - field)
  {
    return false;
  }
  const auto original_size = to.size() < from.size() ? from.size() - to.size() : 0;
  const auto added_size = to.size() > from.size() ? to.size() - from.size() : 0;
  archive.replace(field, from.size(), to);
  if (added_size > 0) {
    std::size_t remaining = added_size;
    std::size_t cursor = field + to.size();
    bool inside_string = false;
    bool escaped = false;
    auto adjusted_json_end = archive.find('}', data);
    while (cursor < adjusted_json_end && remaining > 0) {
      const char character = archive[cursor];
      if (inside_string) {
        if (escaped) {
          escaped = false;
        } else if (character == '\\') {
          escaped = true;
        } else if (character == '"') {
          inside_string = false;
        }
        ++cursor;
      } else if (character == '"') {
        inside_string = true;
        ++cursor;
      } else if (character == ' ') {
        archive.erase(cursor, 1);
        --adjusted_json_end;
        --remaining;
      } else {
        ++cursor;
      }
    }
    return remaining == 0;
  }
  if (original_size > 0) {
    archive.insert(field + to.size(), original_size, ' ');
  }
  return true;
}

bool readArchive(std::string& archive, const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) return false;
  archive.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
  return true;
}

bool writeArchive(const std::string& archive, const std::string& path) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  output.write(archive.data(), static_cast<std::streamsize>(archive.size()));
  return output.good();
}

}  // namespace

// ─── NPZ tests ──────────────────────────────────────────────────────────────

class NpzTest : public ::testing::Test {
 protected:
  ElevationMap map;
  std::string tmp_path;

  void SetUp() override {
    map = ElevationMap(10.0f, 8.0f, 0.5f, "map");

    auto& elev = map.get(layer::elevation);
    for (int i = 0; i < elev.size(); ++i) {
      elev.data()[i] = static_cast<float>(i) * 0.1f;
    }
    elev(0, 0) = NAN;
    elev(1, 1) = NAN;

    tmp_path = testing::TempDir() + "/test_io.npz";
  }

  void TearDown() override { std::remove(tmp_path.c_str()); }
};

TEST_F(NpzTest, RoundTrip) {
  ASSERT_TRUE(io::saveNpz(tmp_path, map));

  ElevationMap loaded;
  ASSERT_TRUE(io::loadNpz(tmp_path, loaded));

  EXPECT_FLOAT_EQ(loaded.getResolution(), map.getResolution());
  EXPECT_EQ(loaded.getSize()(0), map.getSize()(0));
  EXPECT_EQ(loaded.getSize()(1), map.getSize()(1));
  EXPECT_EQ(loaded.getFrameId(), "map");

  auto pos_orig = map.getPosition();
  auto pos_load = loaded.getPosition();
  EXPECT_NEAR(pos_load.x(), pos_orig.x(), 1e-4);
  EXPECT_NEAR(pos_load.y(), pos_orig.y(), 1e-4);

  ASSERT_TRUE(loaded.exists(layer::elevation));
  const auto& orig = map.get(layer::elevation);
  const auto& load = loaded.get(layer::elevation);
  ASSERT_EQ(orig.rows(), load.rows());
  ASSERT_EQ(orig.cols(), load.cols());

  for (int i = 0; i < orig.size(); ++i) {
    if (std::isnan(orig.data()[i])) {
      EXPECT_TRUE(std::isnan(load.data()[i])) << "index " << i;
    } else {
      EXPECT_FLOAT_EQ(orig.data()[i], load.data()[i]) << "index " << i;
    }
  }
}

TEST_F(NpzTest, RoundTripWithStartIndex) {
  map.setStartIndex(nanogrid::Index(5, 3));

  ASSERT_TRUE(io::saveNpz(tmp_path, map));

  ElevationMap loaded;
  ASSERT_TRUE(io::loadNpz(tmp_path, loaded));

  auto start_orig = map.getStartIndex();
  auto start_load = loaded.getStartIndex();
  EXPECT_EQ(start_load(0), start_orig(0));
  EXPECT_EQ(start_load(1), start_orig(1));
}

TEST_F(NpzTest, RoundTripMultipleLayers) {
  map.add("variance");
  auto& var = map.get("variance");
  for (int i = 0; i < var.size(); ++i) {
    var.data()[i] = static_cast<float>(i) * 0.01f;
  }

  ASSERT_TRUE(io::saveNpz(tmp_path, map));

  ElevationMap loaded;
  ASSERT_TRUE(io::loadNpz(tmp_path, loaded));

  ASSERT_TRUE(loaded.exists(layer::elevation));
  ASSERT_TRUE(loaded.exists("variance"));

  const auto& var_orig = map.get("variance");
  const auto& var_load = loaded.get("variance");
  for (int i = 0; i < var_orig.size(); ++i) {
    EXPECT_FLOAT_EQ(var_orig.data()[i], var_load.data()[i]) << "index " << i;
  }
}

TEST_F(NpzTest, SelectiveSave) {
  map.add("variance");
  map.add("intensity");

  ASSERT_TRUE(io::saveNpz(tmp_path, map, {layer::elevation, "variance"}));

  ElevationMap loaded;
  ASSERT_TRUE(io::loadNpz(tmp_path, loaded));

  EXPECT_TRUE(loaded.exists(layer::elevation));
  EXPECT_TRUE(loaded.exists("variance"));
  EXPECT_FALSE(loaded.exists("intensity"));
}

TEST_F(NpzTest, NonexistentLayerSkipped) {
  // Requesting a nonexistent layer should still succeed (skipped with warning)
  ASSERT_TRUE(io::saveNpz(tmp_path, map, {layer::elevation, "no_such_layer"}));

  ElevationMap loaded;
  ASSERT_TRUE(io::loadNpz(tmp_path, loaded));

  EXPECT_TRUE(loaded.exists(layer::elevation));
  EXPECT_FALSE(loaded.exists("no_such_layer"));
}

TEST_F(NpzTest, EmptyMap) {
  // All-NaN map should round-trip correctly
  ElevationMap empty(4.0f, 4.0f, 0.5f, "empty");

  ASSERT_TRUE(io::saveNpz(tmp_path, empty));

  ElevationMap loaded;
  ASSERT_TRUE(io::loadNpz(tmp_path, loaded));

  EXPECT_FLOAT_EQ(loaded.getResolution(), empty.getResolution());
  EXPECT_EQ(loaded.getSize()(0), empty.getSize()(0));

  const auto& elev = loaded.get(layer::elevation);
  for (int i = 0; i < elev.size(); ++i) {
    EXPECT_TRUE(std::isnan(elev.data()[i])) << "index " << i;
  }
}

TEST_F(NpzTest, FutureVersionRejected) {
  // Save a valid file, then patch the version number to a future value
  ASSERT_TRUE(io::saveNpz(tmp_path, map));

  // Read file into memory
  std::ifstream ifs(tmp_path, std::ios::binary);
  std::string content((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
  ifs.close();

  // Replace "version": 1 with "version": 99
  auto pos = content.find("\"version\": 1");
  ASSERT_NE(pos, std::string::npos);
  content.replace(pos, 12, "\"version\":99");

  std::ofstream ofs(tmp_path, std::ios::binary);
  ofs.write(content.data(), content.size());
  ofs.close();

  ElevationMap loaded;
  EXPECT_FALSE(io::loadNpz(tmp_path, loaded));
}

TEST_F(NpzTest, RejectsVersionOutsideIntegerRangeBeforeConversion) {
  ASSERT_TRUE(io::saveNpz(tmp_path, map));
  std::string archive;
  ASSERT_TRUE(readArchive(archive, tmp_path));
  ASSERT_TRUE(replaceMetaTextKeepingSize(
      archive, "\"version\": 1", "\"version\":2147483648"));
  ASSERT_TRUE(writeArchive(archive, tmp_path));

  ElevationMap loaded;
  std::string error;
  EXPECT_FALSE(io::loadNpz(tmp_path, loaded, &error));
  EXPECT_NE(error.find("version"), std::string::npos);
}

TEST_F(NpzTest, RejectsVersionOutsideFloatRangeInsteadOfTreatingItAsAbsent) {
  ASSERT_TRUE(io::saveNpz(tmp_path, map));
  std::string archive;
  ASSERT_TRUE(readArchive(archive, tmp_path));
  ASSERT_TRUE(replaceMetaTextKeepingSize(
      archive, "\"version\": 1", "\"version\":1e100"));
  ASSERT_TRUE(writeArchive(archive, tmp_path));

  ElevationMap loaded;
  std::string error;
  EXPECT_FALSE(io::loadNpz(tmp_path, loaded, &error));
  EXPECT_NE(error.find("version"), std::string::npos);
}

TEST_F(NpzTest, RejectsFractionalVersion) {
  ASSERT_TRUE(io::saveNpz(tmp_path, map));
  std::string archive;
  ASSERT_TRUE(readArchive(archive, tmp_path));
  ASSERT_TRUE(replaceMetaTextKeepingSize(
      archive, "\"version\": 1", "\"version\":0.5"));
  ASSERT_TRUE(writeArchive(archive, tmp_path));

  ElevationMap loaded;
  std::string error;
  EXPECT_FALSE(io::loadNpz(tmp_path, loaded, &error));
  EXPECT_NE(error.find("version"), std::string::npos);
}

TEST_F(NpzTest, RejectsLayerShapeThatDiffersFromMetadata) {
  ASSERT_TRUE(io::saveNpz(tmp_path, map));
  std::string archive;
  ASSERT_TRUE(readArchive(archive, tmp_path));
  ASSERT_TRUE(replaceNpyHeaderText(archive, "elevation.npy", "(20, 16)", "(19, 17)"));
  ASSERT_TRUE(writeArchive(archive, tmp_path));

  ElevationMap loaded;
  std::string error;
  EXPECT_FALSE(io::loadNpz(tmp_path, loaded, &error));
  EXPECT_NE(error.find("unexpected shape"), std::string::npos);
}

TEST_F(NpzTest, RejectsLayerWithUnsupportedElementType) {
  ASSERT_TRUE(io::saveNpz(tmp_path, map));
  std::string archive;
  ASSERT_TRUE(readArchive(archive, tmp_path));
  ASSERT_TRUE(replaceNpyHeaderText(archive, "elevation.npy", "'<f4'", "'<i4'"));
  ASSERT_TRUE(writeArchive(archive, tmp_path));

  ElevationMap loaded;
  std::string error;
  EXPECT_FALSE(io::loadNpz(tmp_path, loaded, &error));
  EXPECT_NE(error.find("float32"), std::string::npos);
}

TEST_F(NpzTest, RejectsStartIndexOutsideMapBounds) {
  ElevationMap small_map(4.0f, 4.0f, 0.5f, "map");
  ASSERT_TRUE(io::saveNpz(tmp_path, small_map));
  std::string archive;
  ASSERT_TRUE(readArchive(archive, tmp_path));
  ASSERT_TRUE(replaceMetaText(
      archive, "\"start_index\": [0, 0]", "\"start_index\": [9, 0]"));
  ASSERT_TRUE(writeArchive(archive, tmp_path));

  ElevationMap loaded;
  std::string error;
  EXPECT_FALSE(io::loadNpz(tmp_path, loaded, &error));
  EXPECT_NE(error.find("start_index"), std::string::npos);
}

TEST_F(NpzTest, LoadNonExistentFile) {
  ElevationMap loaded;
  EXPECT_FALSE(io::loadNpz("/tmp/does_not_exist.npz", loaded));
}

TEST_F(NpzTest, SaveToInvalidPath) {
  EXPECT_FALSE(io::saveNpz("/nonexistent/dir/test.npz", map));
}

// ─── PNG tests ──────────────────────────────────────────────────────────────

class PngTest : public ::testing::Test {
 protected:
  ElevationMap map;
  std::string tmp_path;

  void SetUp() override {
    map = ElevationMap(4.0f, 4.0f, 0.5f, "map");

    auto& elev = map.get(layer::elevation);
    for (int i = 0; i < elev.size(); ++i) {
      elev.data()[i] = static_cast<float>(i) * 0.1f;
    }
    elev(0, 0) = NAN;

    tmp_path = testing::TempDir() + "/test_io.png";
  }

  void TearDown() override { std::remove(tmp_path.c_str()); }

  long fileSize(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    return f.is_open() ? static_cast<long>(f.tellg()) : -1;
  }
};

TEST_F(PngTest, BasicSave) {
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, ColormapViridis) {
  io::PngExportConfig cfg;
  cfg.colormap = io::PngExportConfig::Colormap::VIRIDIS;
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation, cfg));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, ColormapJet) {
  io::PngExportConfig cfg;
  cfg.colormap = io::PngExportConfig::Colormap::JET;
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation, cfg));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, ColormapGrayscale) {
  io::PngExportConfig cfg;
  cfg.colormap = io::PngExportConfig::Colormap::GRAYSCALE;
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation, cfg));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, NormalizeFixedRange) {
  io::PngExportConfig cfg;
  cfg.normalize = io::PngExportConfig::Normalize::FIXED_RANGE;
  cfg.fixed_min = 0.0f;
  cfg.fixed_max = 5.0f;
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation, cfg));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, NormalizeMinMax) {
  io::PngExportConfig cfg;
  cfg.normalize = io::PngExportConfig::Normalize::MIN_MAX;
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation, cfg));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, AlignToWorldDisabled) {
  map.setStartIndex(nanogrid::Index(3, 2));
  io::PngExportConfig cfg;
  cfg.align_to_world = false;
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation, cfg));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, EmptyMap) {
  ElevationMap empty(4.0f, 4.0f, 0.5f, "empty");
  ASSERT_TRUE(io::savePng(tmp_path, empty, layer::elevation));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, NonexistentLayer) {
  EXPECT_FALSE(io::savePng(tmp_path, map, "no_such_layer"));
}

TEST_F(PngTest, SaveToInvalidPath) {
  EXPECT_FALSE(io::savePng("/nonexistent/dir/test.png", map, layer::elevation));
}
