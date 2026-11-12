//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "pub_handler.h"
#include <algorithm>
#include <cstdlib>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <limits>
#include <utility>

namespace livox_ros {

namespace
{
constexpr auto kMaxQueuedPacketAge = std::chrono::milliseconds(250);
constexpr std::size_t kMaxRawPacketQueueSize = 512;
constexpr auto kQueueLogPeriod = std::chrono::seconds(1);
}

std::atomic<bool> PubHandler::is_timestamp_sync_;

PubHandler &pub_handler() {
  static PubHandler handler;
  return handler;
}

void PubHandler::Init() {
}

void PubHandler::Uninit() {
  if (lidar_listen_id_ > 0) {
    LivoxLidarRemovePointCloudObserver(lidar_listen_id_);
    lidar_listen_id_ = 0;
  }

  RequestExit();

  if (point_process_thread_ &&
    point_process_thread_->joinable()) {
    point_process_thread_->join();
    point_process_thread_ = nullptr;
  } else {
    /* */
  }
}

void PubHandler::RequestExit() {
  is_quit_.store(true);
}

void PubHandler::SetPointCloudConfig(const double publish_freq) {
  publish_interval_ = (kNsPerSecond / (publish_freq * 10)) * 10;
  publish_interval_tolerance_ = publish_interval_ - kNsTolerantFrameTimeDeviation;
  publish_interval_ms_ = publish_interval_ / kRatioOfMsToNs;
  if (!point_process_thread_) {
    point_process_thread_ = std::make_shared<std::thread>(&PubHandler::RawDataProcess, this);
  }
  return;
}

void PubHandler::SetImuDataCallback(ImuDataCallback cb, void* client_data) {
  imu_client_data_ = client_data;
  imu_callback_ = cb;
}

void PubHandler::AddLidarsExtParam(LidarExtParameter& lidar_param) {
  std::unique_lock<std::mutex> lock(packet_mutex_);
  uint32_t id = 0;
  GetLidarId(lidar_param.lidar_type, lidar_param.handle, id);
  lidar_extrinsics_[id] = lidar_param;
}

void PubHandler::ClearAllLidarsExtrinsicParams() {
  std::unique_lock<std::mutex> lock(packet_mutex_);
  lidar_extrinsics_.clear();
}

void PubHandler::SetPointCloudsCallback(PointCloudsCallback cb, void* client_data) {
  pub_client_data_ = client_data;
  points_callback_ = cb;
  lidar_listen_id_ = LivoxLidarAddPointCloudObserver(OnLivoxLidarPointCloudCallback, this);
}

void PubHandler::OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type,
                                                LivoxLidarEthernetPacket *data, void *client_data) {
  PubHandler* self = (PubHandler*)client_data;
  if (!self) {
    return;
  }

  if (data->time_type != kTimestampTypeNoSync) {
    is_timestamp_sync_.store(true);
  } else {
    is_timestamp_sync_.store(false);
  }

  if (data->data_type == kLivoxLidarImuData) {
    if (self->imu_callback_) {
      RawImuPoint* imu = (RawImuPoint*) data->data;
      ImuData imu_data;
      imu_data.lidar_type = static_cast<uint8_t>(LidarProtoType::kLivoxLidarType);
      imu_data.handle = handle;
      imu_data.time_stamp = GetEthPacketTimestamp(data->time_type,
                                                  data->timestamp, sizeof(data->timestamp));
      imu_data.gyro_x = imu->gyro_x;
      imu_data.gyro_y = imu->gyro_y;
      imu_data.gyro_z = imu->gyro_z;
      imu_data.acc_x = imu->acc_x;
      imu_data.acc_y = imu->acc_y;
      imu_data.acc_z = imu->acc_z;
      self->imu_callback_(&imu_data, self->imu_client_data_);
    }
    return;
  }
  if (data->dot_num == 0) {
    return;
  }
  RawPacket packet = {};
  packet.handle = handle;
  packet.lidar_type = LidarProtoType::kLivoxLidarType;
  packet.extrinsic_enable = false; 
  if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeIndustrialHAP) {
    packet.line_num = kLineNumberHAP;
  } else if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360) {
    packet.line_num = kLineNumberMid360;
  } else {
    packet.line_num = kLineNumberDefault;
  }
  packet.data_type = data->data_type;
  packet.point_num = data->dot_num;
  packet.point_interval = data->time_interval * 100 / data->dot_num;  //ns
  packet.time_stamp = GetEthPacketTimestamp(data->time_type,
                                            data->timestamp, sizeof(data->timestamp));
  packet.received_at = std::chrono::steady_clock::now();
  packet.source_age_valid = data->time_type == kTimestampTypeGptpOrPtp;
  if (packet.source_age_valid) {
    const auto system_now = std::chrono::system_clock::now().time_since_epoch();
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(system_now).count();
    packet.source_age_at_receive_ns = now_ns - static_cast<int64_t>(packet.time_stamp);
    self->last_source_age_at_receive_ns_.store(packet.source_age_at_receive_ns);
    self->last_source_age_valid_.store(true);
    if (packet.source_age_at_receive_ns >
      std::chrono::duration_cast<std::chrono::nanoseconds>(kMaxQueuedPacketAge).count())
    {
      self->dropped_source_stale_packets_.fetch_add(1);
      self->reset_accumulator_requested_.store(true);
      self->packet_condition_.notify_one();
      return;
    }
  }
  uint32_t length = data->length - sizeof(LivoxLidarEthernetPacket) + 1;
  packet.raw_data.reserve(length);
  packet.raw_data.insert(packet.raw_data.end(), data->data, data->data + length);
  {
    std::unique_lock<std::mutex> lock(self->packet_mutex_);
    if (self->raw_packet_queue_.size() >= kMaxRawPacketQueueSize) {
      const auto overflow = self->raw_packet_queue_.size() - kMaxRawPacketQueueSize + 1;
      for (std::size_t i = 0; i < overflow; ++i) self->raw_packet_queue_.pop_front();
      self->dropped_overflow_packets_.fetch_add(overflow);
      self->reset_accumulator_requested_.store(true);
    }
    self->raw_packet_queue_.push_back(std::move(packet));
  }
  self->packet_condition_.notify_one();

  return;
}

void PubHandler::PublishPointCloud() {
  //publish point
  if (points_callback_) {
    points_callback_(&frame_, pub_client_data_);
  }
  return;
}

void PubHandler::LogPointBatch(
    const char *trigger, uint64_t skipped_periods, double period_phase_ms) {
  ++published_batch_count_;
  for (uint8_t i = 0; i < frame_.lidar_num; ++i) {
    const PointPacket &packet = frame_.lidar_point[i];
    if (packet.points_num == 0 || packet.points == nullptr) {
      continue;
    }

    uint64_t min_offset_ns = std::numeric_limits<uint64_t>::max();
    uint64_t max_offset_ns = 0;
    for (uint32_t point_index = 0; point_index < packet.points_num; ++point_index) {
      min_offset_ns = std::min(min_offset_ns, packet.points[point_index].offset_time);
      max_offset_ns = std::max(max_offset_ns, packet.points[point_index].offset_time);
    }
    const uint64_t packet_count = packets_since_publish_[packet.handle];
    const auto now = std::chrono::steady_clock::now();
    const bool sparse_log_due = packet.points_num < 1000 &&
      (last_sparse_batch_log_.time_since_epoch().count() == 0 ||
      now - last_sparse_batch_log_ >= std::chrono::seconds(1));
    if (sparse_log_due || published_batch_count_ % 10 == 0) {
      std::cerr << "Livox published scan batch: batch=" << published_batch_count_
                << " trigger=" << trigger
                << " lidar=" << packet.handle
                << " points=" << packet.points_num
                << " raw_packets=" << packet_count
                << " source_span_ms="
                << (max_offset_ns - min_offset_ns) / 1000000.0
                << " skipped_periods=" << skipped_periods
                << " period_phase_ms=" << period_phase_ms << std::endl;
      if (sparse_log_due) {
        last_sparse_batch_log_ = now;
      }
    }
    packets_since_publish_[packet.handle] = 0;
  }
}

void PubHandler::CheckTimer(uint32_t id, TimePoint packet_received_at) {
  ++packets_since_publish_[id];

  if (PubHandler::is_timestamp_sync_.load()) { // Enable time synchronization
    auto& process_handler = lidar_process_handlers_[id];
    uint64_t recent_time_ms = process_handler->GetRecentTimeStamp() / kRatioOfMsToNs;
    if ((recent_time_ms % publish_interval_ms_ != 0) || recent_time_ms == 0) {
      return;
    }

    uint64_t diff = process_handler->GetRecentTimeStamp() - process_handler->GetLidarBaseTime();
    if (diff < publish_interval_tolerance_) {
      return;
    }

    frame_.base_time[frame_.lidar_num] = process_handler->GetLidarBaseTime();
    points_[id].clear();
    process_handler->GetLidarPointClouds(points_[id]);
    if (points_[id].empty()) {
      return;
    }
    PointPacket& lidar_point = frame_.lidar_point[frame_.lidar_num];
    lidar_point.lidar_type = LidarProtoType::kLivoxLidarType;  // TODO:
    lidar_point.handle = id;
    lidar_point.points_num = points_[id].size();
    lidar_point.points = points_[id].data();
    lidar_point.source_time_valid = process_handler->HasValidSourceTimestamp();
    frame_.lidar_num++;
    
    if (frame_.lidar_num != 0) {
      LogPointBatch("source_time_boundary", 0, 0.0);
      PublishPointCloud();
      frame_.lidar_num = 0;
    }
  } else { // Disable time synchronization
    const auto now_time = std::chrono::steady_clock::now();
    const auto publish_interval = std::chrono::duration_cast<TimePoint::duration>(
      std::chrono::nanoseconds(publish_interval_));
    if (publish_interval <= TimePoint::duration::zero()) {
      return;
    }
    point_batch_windows_[id].observePacket(packet_received_at);
    if (!have_last_pub_time_) {
      last_pub_time_ = now_time;
      have_last_pub_time_ = true;
      return;
    }
    const auto elapsed = now_time - last_pub_time_;
    if (elapsed < publish_interval) {
      return;
    }
    const auto completed_periods = elapsed / publish_interval;
    const auto skipped_periods = completed_periods > 0 ? completed_periods - 1 : 0;
    const auto minimum_coverage = std::chrono::duration_cast<TimePoint::duration>(
      std::chrono::nanoseconds(publish_interval_tolerance_));
    const auto maximum_batch_wait = publish_interval +
      std::chrono::duration_cast<TimePoint::duration>(kMaxQueuedPacketAge);
    bool published_batch = false;
    for (auto &process_handler : lidar_process_handlers_) {
      uint32_t handle = process_handler.first;
      const auto window = point_batch_windows_.find(handle);
      if (window == point_batch_windows_.end() ||
        !window->second.canPublish(
          now_time, minimum_coverage, publish_interval, maximum_batch_wait))
      {
        continue;
      }
      frame_.base_time[frame_.lidar_num] = process_handler.second->GetLidarBaseTime();
      points_[handle].clear();
      process_handler.second->GetLidarPointClouds(points_[handle]);
      if (points_[handle].empty()) {
        window->second.markPublished(now_time);
        packets_since_publish_[handle] = 0;
        continue;
      }
      PointPacket& lidar_point = frame_.lidar_point[frame_.lidar_num];
      lidar_point.lidar_type = LidarProtoType::kLivoxLidarType;  // TODO:
      lidar_point.handle = handle;
      lidar_point.points_num = points_[handle].size();
      lidar_point.points = points_[handle].data();
      lidar_point.source_time_valid = process_handler.second->HasValidSourceTimestamp();
      frame_.lidar_num++;
      window->second.markPublished(now_time);
      published_batch = true;
    }
    if (published_batch && frame_.lidar_num > 0) {
      const double period_phase_ms = std::chrono::duration<double, std::milli>(
        elapsed - publish_interval * completed_periods).count();
      LogPointBatch("period_elapsed", skipped_periods, period_phase_ms);
      PublishPointCloud();
      // Rebase on this actual drain. Advancing by nominal periods leaves the
      // remainder as the next interval, which can publish another short batch.
      last_pub_time_ = now_time;
    }
    frame_.lidar_num = 0;
  }
  return;
}

void PubHandler::resetAccumulatedPointClouds(TimePoint now, const char * reason)
{
  for (auto &entry : lidar_process_handlers_) {
    entry.second->ResetPointClouds();
  }
  for (auto &window : point_batch_windows_) {
    window.second.reset(now);
  }
  packets_since_publish_.clear();
  frame_.lidar_num = 0;
  last_pub_time_ = now;
  have_last_pub_time_ = true;
  if (reason != nullptr) {
    std::cerr << "Livox point batch reset: reason=" << reason << std::endl;
  }
}

void PubHandler::RawDataProcess() {
  RawPacket raw_data;
  last_pub_time_ = std::chrono::steady_clock::now();
  have_last_pub_time_ = true;
  auto next_queue_log = std::chrono::steady_clock::now() + kQueueLogPeriod;
  while (!is_quit_.load()) {
    bool have_packet = false;
    std::size_t queue_depth = 0;
    std::size_t stale_drops = 0;
    {
      std::unique_lock<std::mutex> lock(packet_mutex_);
      if (raw_packet_queue_.empty()) {
        packet_condition_.wait_for(lock, std::chrono::milliseconds(500));
      }
      const auto now = std::chrono::steady_clock::now();
      while (!raw_packet_queue_.empty() &&
        now - raw_packet_queue_.front().received_at > kMaxQueuedPacketAge)
      {
        raw_packet_queue_.pop_front();
        ++stale_drops;
      }
      if (stale_drops > 0) {
        dropped_stale_packets_.fetch_add(stale_drops);
        reset_accumulator_requested_.store(true);
      }
      if (!raw_packet_queue_.empty()) {
        raw_data = std::move(raw_packet_queue_.front());
        raw_packet_queue_.pop_front();
        queue_depth = raw_packet_queue_.size();
        have_packet = true;
      }
    }
    if (reset_accumulator_requested_.exchange(false)) {
      resetAccumulatedPointClouds(
        std::chrono::steady_clock::now(), "queue_or_source_stale");
    }
    if (!have_packet) {
      if (std::chrono::steady_clock::now() >= next_queue_log) {
        std::cerr << "Livox point queue: depth=" << queue_depth
                  << " last_source_age_ms="
                  << (last_source_age_valid_.load() ?
                    last_source_age_at_receive_ns_.load() / 1000000 : -1)
                  << " source_stale_drops=" << dropped_source_stale_packets_.load()
                  << " stale_drops=" << dropped_stale_packets_.load()
                  << " overflow_drops=" << dropped_overflow_packets_.load() << std::endl;
        next_queue_log = std::chrono::steady_clock::now() + kQueueLogPeriod;
      }
      continue;
    }
    uint32_t id = 0;
    GetLidarId(raw_data.lidar_type, raw_data.handle, id);
    if (!PubHandler::is_timestamp_sync_.load()) {
      auto &window = point_batch_windows_[id];
      const auto now = std::chrono::steady_clock::now();
      const auto maximum_batch_wait = std::chrono::duration_cast<TimePoint::duration>(
        std::chrono::nanoseconds(publish_interval_) +
        std::chrono::duration_cast<std::chrono::nanoseconds>(kMaxQueuedPacketAge));
      if (window.shouldResetBeforePacket(
          raw_data.received_at, now,
          std::chrono::duration_cast<TimePoint::duration>(kMaxQueuedPacketAge),
          maximum_batch_wait))
      {
        resetAccumulatedPointClouds(now, "stream_gap_or_batch_age");
      }
    }
    if (lidar_process_handlers_.find(id) == lidar_process_handlers_.end()) {
      lidar_process_handlers_[id].reset(new LidarPubHandler());
    }
    auto &process_handler = lidar_process_handlers_[id];
    if (lidar_extrinsics_.find(id) != lidar_extrinsics_.end()) {
        lidar_process_handlers_[id]->SetLidarsExtParam(lidar_extrinsics_[id]);
    }
    if (std::chrono::steady_clock::now() >= next_queue_log) {
      const auto packet_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - raw_data.received_at).count();
      std::cerr << "Livox point queue: depth=" << queue_depth
                << " packet_age_ms=" << packet_age_ms
                << " source_age_ms="
                << (raw_data.source_age_valid ? raw_data.source_age_at_receive_ns / 1000000 : -1)
                << " source_stale_drops=" << dropped_source_stale_packets_.load()
                << " stale_drops=" << dropped_stale_packets_.load()
                << " overflow_drops=" << dropped_overflow_packets_.load() << std::endl;
      next_queue_log = std::chrono::steady_clock::now() + kQueueLogPeriod;
    }
    if (raw_data.source_age_valid) {
      const auto system_now = std::chrono::system_clock::now().time_since_epoch();
      const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(system_now).count();
      const auto source_age_ns = now_ns - static_cast<int64_t>(raw_data.time_stamp);
      if (source_age_ns >
        std::chrono::duration_cast<std::chrono::nanoseconds>(kMaxQueuedPacketAge).count())
      {
        dropped_source_stale_packets_.fetch_add(1);
        reset_accumulator_requested_.store(true);
        continue;
      }
    }
    process_handler->PointCloudProcess(raw_data);
    CheckTimer(id, raw_data.received_at);
  }
}

bool PubHandler::GetLidarId(LidarProtoType lidar_type, uint32_t handle, uint32_t& id) {
  if (lidar_type == kLivoxLidarType) {
    id = handle;
    return true;
  }
  return false;
}

uint64_t PubHandler::GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size) {
  LdsStamp time;
  memcpy(time.stamp_bytes, time_stamp, size);

  if (timestamp_type == kTimestampTypeGptpOrPtp ||
      timestamp_type == kTimestampTypeGps) {
    return time.stamp;
  }

  return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

/*******************************/
/*  LidarPubHandler Definitions*/
LidarPubHandler::LidarPubHandler() : is_set_extrinsic_params_(false) {}

uint64_t LidarPubHandler::GetLidarBaseTime() {
  if (points_clouds_.empty()) {
    return 0;
  }
  return points_clouds_.at(0).offset_time;
}

void LidarPubHandler::GetLidarPointClouds(std::vector<PointXyzlt>& points_clouds) {
  points_clouds.swap(points_clouds_);
}

void LidarPubHandler::ResetPointClouds() {
  points_clouds_.clear();
}

uint64_t LidarPubHandler::GetRecentTimeStamp() {
  if (points_clouds_.empty()) {
    return 0;
  }
  return points_clouds_.back().offset_time;
}

uint32_t LidarPubHandler::GetLidarPointCloudsSize() {
  return points_clouds_.size();
}

//convert to standard format and extrinsic compensate
void LidarPubHandler::PointCloudProcess(RawPacket & pkt) {
  source_time_valid_ = pkt.source_age_valid;
  if (pkt.lidar_type == LidarProtoType::kLivoxLidarType) {
    LivoxLidarPointCloudProcess(pkt);
  } else {
    static bool flag = false;
    if (!flag) {
      std::cout << "error, unsupported protocol type: " << static_cast<int>(pkt.lidar_type) << std::endl;
      flag = true;      
    }
  }
}

void LidarPubHandler::LivoxLidarPointCloudProcess(RawPacket & pkt) {
  switch (pkt.data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
      ProcessCartesianHighPoint(pkt);
      break;
    case kLivoxLidarCartesianCoordinateLowData:
      ProcessCartesianLowPoint(pkt);
      break;
    case kLivoxLidarSphericalCoordinateData:
      ProcessSphericalPoint(pkt);
      break;
    default:
      std::cout << "unknown data type: " << static_cast<int>(pkt.data_type)
                << " !!" << std::endl;
      break;
  }
}

void LidarPubHandler::SetLidarsExtParam(LidarExtParameter lidar_param) {
  if (is_set_extrinsic_params_) {
    return;
  }
  extrinsic_.trans[0] = lidar_param.param.x;
  extrinsic_.trans[1] = lidar_param.param.y;
  extrinsic_.trans[2] = lidar_param.param.z;

  double cos_roll = cos(static_cast<double>(lidar_param.param.roll * PI / 180.0));
  double cos_pitch = cos(static_cast<double>(lidar_param.param.pitch * PI / 180.0));
  double cos_yaw = cos(static_cast<double>(lidar_param.param.yaw * PI / 180.0));
  double sin_roll = sin(static_cast<double>(lidar_param.param.roll * PI / 180.0));
  double sin_pitch = sin(static_cast<double>(lidar_param.param.pitch * PI / 180.0));
  double sin_yaw = sin(static_cast<double>(lidar_param.param.yaw * PI / 180.0));

  extrinsic_.rotation[0][0] = cos_pitch * cos_yaw;
  extrinsic_.rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  extrinsic_.rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

  extrinsic_.rotation[1][0] = cos_pitch * sin_yaw;
  extrinsic_.rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  extrinsic_.rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

  extrinsic_.rotation[2][0] = -sin_pitch;
  extrinsic_.rotation[2][1] = sin_roll * cos_pitch;
  extrinsic_.rotation[2][2] = cos_roll * cos_pitch;

  is_set_extrinsic_params_ = true;
}

void LidarPubHandler::ProcessCartesianHighPoint(RawPacket & pkt) {
  LivoxLidarCartesianHighRawPoint* raw = (LivoxLidarCartesianHighRawPoint*)pkt.raw_data.data();
  PointXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 1000.0;
      point.y = raw[i].y / 1000.0;
      point.z = raw[i].z / 1000.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 1000.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 1000.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 1000.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ProcessCartesianLowPoint(RawPacket & pkt) {
  LivoxLidarCartesianLowRawPoint* raw = (LivoxLidarCartesianLowRawPoint*)pkt.raw_data.data();
  PointXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 100.0;
      point.y = raw[i].y / 100.0;
      point.z = raw[i].z / 100.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 100.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 100.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 100.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ProcessSphericalPoint(RawPacket& pkt) {
  LivoxLidarSpherPoint* raw = (LivoxLidarSpherPoint*)pkt.raw_data.data();
  PointXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    double radius = raw[i].depth / 1000.0;
    double theta = raw[i].theta / 100.0 / 180 * PI;
    double phi = raw[i].phi / 100.0 / 180 * PI;
    double src_x = radius * sin(theta) * cos(phi);
    double src_y = radius * sin(theta) * sin(phi);
    double src_z = radius * cos(theta);
    if (pkt.extrinsic_enable) {
      point.x = src_x;
      point.y = src_y;
      point.z = src_z;
    } else {
      point.x = src_x * extrinsic_.rotation[0][0] +
                src_y * extrinsic_.rotation[0][1] +
                src_z * extrinsic_.rotation[0][2] + (extrinsic_.trans[0] / 1000.0);
      point.y = src_x * extrinsic_.rotation[1][0] +
                src_y * extrinsic_.rotation[1][1] +
                src_z * extrinsic_.rotation[1][2] + (extrinsic_.trans[1] / 1000.0);
      point.z = src_x * extrinsic_.rotation[2][0] +
                src_y * extrinsic_.rotation[2][1] +
                src_z * extrinsic_.rotation[2][2] + (extrinsic_.trans[2] / 1000.0);
    }

    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    points_clouds_.push_back(point);
  }
}
} // namespace livox_ros
