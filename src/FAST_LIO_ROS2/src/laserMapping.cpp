// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <algorithm>
#include <array>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <cstdint>
#include <atomic>
#include <limits>
#include <stdexcept>
#include <vector>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <fast_lio/scan_health.hpp>
#include <fast_lio/recovery_gate.hpp>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = false, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0;
std::atomic<int> publish_count{0};
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool    is_first_lidar = true;
/*** Every abnormality the frame being processed tripped, as ScanFaultReason bits. ***/
std::uint32_t scan_fault_reasons = fast_lio::SCAN_FAULT_NONE;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_fastlio_health;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_fastlio_recovery_state;

void publish_fastlio_health(bool healthy)
{
    if (!pub_fastlio_health) return;
    std_msgs::msg::Bool message;
    message.data = healthy;
    pub_fastlio_health->publish(message);
}

void publish_fastlio_recovery_state(const std::string &state)
{
    if (!pub_fastlio_recovery_state) return;
    std_msgs::msg::String message;
    message.data = state;
    pub_fastlio_recovery_state->publish(message);
}

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
M3D input_level_rotation = M3D::Identity();
std::unique_ptr<tf2_ros::StaticTransformBroadcaster> raw_body_tf_broadcaster_;
using LidarScan = fast_lio::LidarScan<PointCloudXYZI::Ptr>;

constexpr std::size_t MAX_LIDAR_BUFFER_SIZE = 5;
constexpr std::size_t MAX_IMU_BUFFER_SIZE = 200;
double health_min_effective_ratio = 0.02;
int health_min_effective_points = 10;
double health_max_mean_residual = 0.15;
double health_max_translation_correction = 0.30;
double health_max_rotation_correction = 0.35;
double health_max_speed = 1.5;
double health_max_vertical_speed = 0.5;
double health_max_imu_gap = 0.05;
double health_max_lidar_gap = 0.50;
double health_reinit_imu_gap = 0.30;
double health_reinit_lidar_gap = 1.00;
double health_max_processing_lag = 0.60;
int health_max_consecutive_faults = 10;
double health_min_consecutive_fault_duration = 1.0;
bool health_shutdown_on_fault = false;
bool health_shutdown_on_relocalization_lock = true;
/*** Raised by the sensor callbacks, consumed once per frame by the processing
     thread. They report lost samples, never plain compute latency. ***/
std::atomic<bool> imu_gap_detected{false};
std::atomic<bool> lidar_queue_drop_detected{false};
std::atomic<bool> imu_reinit_gap_detected{false};
std::atomic<bool> lidar_reinit_gap_detected{false};
std::atomic<bool> sensor_time_regression{false};
std::atomic<double> last_imu_gap{0.0};
std::atomic<double> last_lidar_gap{0.0};
std::atomic<double> last_imu_receive_gap{0.0};
std::atomic<double> last_lidar_receive_gap{0.0};
std::atomic<std::uint64_t> dropped_lidar_scans{0};
std::atomic<std::uint64_t> dropped_imu_samples{0};
std::chrono::steady_clock::time_point last_imu_receive_time{};
std::chrono::steady_clock::time_point last_lidar_receive_time{};
double previous_imu_timestamp = -1.0;
double previous_processed_lidar_timestamp = -1.0;
std::chrono::steady_clock::time_point current_scan_received_at;
std::size_t current_scan_raw_point_count = 0;
std::size_t current_scan_preprocessed_point_count = 0;
fast_lio::ScanHealthMonitor scan_health_monitor;
fast_lio::RecoveryGate odometry_recovery_gate;
state_ikfom current_scan_prediction;
esekfom::esekf<state_ikfom, 12, input_ikfom>::cov current_scan_prediction_covariance =
    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov::Identity();
bool have_current_scan_prediction = false;
bool has_last_good_state = false;
bool recovery_required = false;
bool recovery_requires_relocalization = false;
bool prime_imu_time_axis_after_reset = false;

fast_lio::LidarScanQueue<PointCloudXYZI::Ptr> lidar_queue(MAX_LIDAR_BUFFER_SIZE);
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr complete_map(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

/*** Last state that passed every gate. A non-finite estimate cannot be recovered by
     re-initialising in place, so the filter falls back to this instead. ***/
state_ikfom last_good_state;
esekfom::esekf<state_ikfom, 12, input_ikfom>::cov last_good_covariance =
    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov::Identity();

bool ekf_state_is_finite(const state_ikfom &state)
{
    return state.pos.allFinite() && state.vel.allFinite() &&
           state.rot.toRotationMatrix().allFinite();
}

/*** Keep the published pose derived from one single state. publish_odometry() reads
     the position from state_point and the orientation from geoQuat, so both have to
     be refreshed together or a rejected frame mixes two different frames' poses. ***/
void refresh_output_state()
{
    euler_cur = SO3ToEuler(state_point.rot);
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
    geoQuat.x = state_point.rot.coeffs()[0];
    geoQuat.y = state_point.rot.coeffs()[1];
    geoQuat.z = state_point.rot.coeffs()[2];
    geoQuat.w = state_point.rot.coeffs()[3];
}

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void append_complete_scan_to_map()
{
    for (const PointType & point_body : feats_undistort->points)
    {
        if (!std::isfinite(point_body.x) || !std::isfinite(point_body.y) ||
            !std::isfinite(point_body.z))
        {
            continue;
        }

        PointType point_world;
        pointBodyToWorld(&point_body, &point_world);
        complete_map->push_back(point_world);
    }
    complete_map->width = static_cast<std::uint32_t>(complete_map->points.size());
    complete_map->height = 1;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    const auto callback_received_at = std::chrono::steady_clock::now();
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);

    std::lock_guard<std::mutex> lock(mtx_buffer);
    scan_count ++;
    if (last_lidar_receive_time.time_since_epoch().count() != 0) {
        last_lidar_receive_gap.store(std::chrono::duration<double>(
            callback_received_at - last_lidar_receive_time).count());
    }
    last_lidar_receive_time = callback_received_at;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar timestamp went backwards, clearing buffers" << std::endl;
        lidar_queue.clear();
        sensor_time_regression.store(true);
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    if (last_timestamp_lidar > 0.0 && cur_time - last_timestamp_lidar > health_max_lidar_gap)
    {
        if (cur_time - last_timestamp_lidar > health_reinit_lidar_gap)
        {
            lidar_reinit_gap_detected.store(true);
        }
    }
    const auto queue_drops = lidar_queue.push({
        ptr, cur_time, callback_received_at,
        static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height)});
    if (queue_drops > 0)
    {
        dropped_lidar_scans.fetch_add(queue_drops);
        lidar_queue_drop_detected.store(true);
    }
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) 
{
    const auto callback_received_at = std::chrono::steady_clock::now();
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);

    std::lock_guard<std::mutex> lock(mtx_buffer);
    scan_count ++;
    if (last_lidar_receive_time.time_since_epoch().count() != 0) {
        last_lidar_receive_gap.store(std::chrono::duration<double>(
            callback_received_at - last_lidar_receive_time).count());
    }
    last_lidar_receive_time = callback_received_at;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar timestamp went backwards, clearing buffers" << std::endl;
        lidar_queue.clear();
        sensor_time_regression.store(true);
    }
    if(is_first_lidar)
    {
        is_first_lidar = false;
    }
    if (last_timestamp_lidar > 0.0 && cur_time - last_timestamp_lidar > health_max_lidar_gap)
    {
        if (cur_time - last_timestamp_lidar > health_reinit_lidar_gap)
        {
            lidar_reinit_gap_detected.store(true);
        }
    }
    last_timestamp_lidar = cur_time;
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_queue.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    const auto queue_drops = lidar_queue.push({
        ptr, last_timestamp_lidar, callback_received_at,
        static_cast<std::size_t>(msg->point_num)});
    if (queue_drops > 0)
    {
        dropped_lidar_scans.fetch_add(queue_drops);
        lidar_queue_drop_detected.store(true);
    }


    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    const auto callback_received_at = std::chrono::steady_clock::now();
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    const V3D raw_acc(msg_in->linear_acceleration.x,
                      msg_in->linear_acceleration.y,
                      msg_in->linear_acceleration.z);
    const V3D raw_gyr(msg_in->angular_velocity.x,
                      msg_in->angular_velocity.y,
                      msg_in->angular_velocity.z);
    const V3D level_acc = input_level_rotation * raw_acc;
    const V3D level_gyr = input_level_rotation * raw_gyr;
    msg->linear_acceleration.x = level_acc.x();
    msg->linear_acceleration.y = level_acc.y();
    msg->linear_acceleration.z = level_acc.z();
    msg->angular_velocity.x = level_gyr.x();
    msg->angular_velocity.y = level_gyr.y();
    msg->angular_velocity.z = level_gyr.z();
    

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    double lidar_imu_sync_diff = 0.0;
    {
        std::lock_guard<std::mutex> lock(mtx_buffer);
        lidar_imu_sync_diff = timediff_lidar_wrt_imu;
    }
    if (abs(lidar_imu_sync_diff) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        rclcpp::Time(lidar_imu_sync_diff + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    std::lock_guard<std::mutex> lock(mtx_buffer);

    if (last_imu_receive_time.time_since_epoch().count() != 0) {
        last_imu_receive_gap.store(std::chrono::duration<double>(
            callback_received_at - last_imu_receive_time).count());
    }
    last_imu_receive_time = callback_received_at;

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "imu timestamp went backwards, clearing buffer" << std::endl;
        imu_buffer.clear();
        sensor_time_regression.store(true);
    }

    if (previous_imu_timestamp > 0.0 && timestamp - previous_imu_timestamp > health_max_imu_gap)
    {
        imu_gap_detected.store(true);
        last_imu_gap.store(timestamp - previous_imu_timestamp);
        if (timestamp - previous_imu_timestamp > health_reinit_imu_gap)
        {
            imu_reinit_gap_detected.store(true);
        }
    }
    previous_imu_timestamp = timestamp;
    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    while (imu_buffer.size() > MAX_IMU_BUFFER_SIZE)
    {
        imu_buffer.pop_front();
        dropped_imu_samples.fetch_add(1);
        imu_gap_detected.store(true);
        last_imu_gap.store(health_max_imu_gap);
    }
    sig_buffer.notify_all();
}

void publish_raw_body_tf(const rclcpp::Time &stamp)
{
    Eigen::Quaterniond rotation(input_level_rotation);
    rotation.normalize();

    geometry_msgs::msg::TransformStamped transform;
    const int64_t stamp_nanoseconds = stamp.nanoseconds();
    transform.header.stamp.sec = static_cast<int32_t>(
        stamp_nanoseconds / 1000000000LL);
    transform.header.stamp.nanosec = static_cast<uint32_t>(
        stamp_nanoseconds % 1000000000LL);
    transform.header.frame_id = "body";
    transform.child_frame_id = "body_raw";
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();
    raw_body_tf_broadcaster_->sendTransform(transform);
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);

    /*** Claim a lidar scan. Claiming moves it into the queue's pending slot, so a
         callback overflow can no longer drop the scan that is waiting for IMU. ***/
    if (!lidar_queue.has_pending())
    {
        if (!lidar_queue.claim()) return false;

        const PointCloudXYZI::Ptr &cloud = lidar_queue.pending().cloud;
        const double scan_beg_time = lidar_queue.pending().timestamp;
        if (cloud->points.size() <= 1) // time too little
        {
            lidar_end_time = scan_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (cloud->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = scan_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = scan_beg_time + cloud->points.back().curvature / double(1000);
            lidar_mean_scantime += (cloud->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }
    }

    const auto & pending_scan = lidar_queue.pending();
    meas.lidar = pending_scan.cloud;
    meas.lidar_beg_time = pending_scan.timestamp;
    current_scan_raw_point_count = pending_scan.raw_point_count;
    current_scan_preprocessed_point_count = pending_scan.cloud ?
        pending_scan.cloud->points.size() : 0;
    meas.lidar_end_time = lidar_end_time;

    if (imu_buffer.empty() || last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    current_scan_received_at = lidar_queue.pending().received_at;
    lidar_queue.release_pending();
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if(scan_pub_en && pubLaserCloudFull->get_subscription_count() > 0)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "odom";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

}

void publish_frame_body(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudClearance)
{
    PointCloudXYZI::Ptr laserCloudFullRes(feats_undistort);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&laserCloudFullRes->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);

    if (pubLaserCloudClearance && pubLaserCloudClearance->get_subscription_count() > 0)
    {
        constexpr std::size_t kMaximumClearanceRaysPerScan = 5000;
        const auto & point_motions = p_imu->pointBodyMotionInScanEnd();
        const std::size_t point_count = laserCloudIMUBody->points.size();
        if (point_motions.size() == point_count)
        {
            const std::size_t stride = std::max<std::size_t>(
                1, (point_count + kMaximumClearanceRaysPerScan - 1) /
                kMaximumClearanceRaysPerScan);
            std::vector<std::array<float, 7>> clearance_points;
            clearance_points.reserve(std::min(point_count, kMaximumClearanceRaysPerScan));
            const std::array<double, 3> current_lidar_translation{{
                state_point.offset_T_L_I.x(),
                state_point.offset_T_L_I.y(),
                state_point.offset_T_L_I.z()}};
            for (std::size_t index = 0; index < point_count; index += stride)
            {
                if (!point_motions[index].valid) continue;
                const auto & point = laserCloudIMUBody->points[index];
                if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                    !std::isfinite(point.z)) continue;
                const auto origin = fast_lio::sensorOriginInScanEndBody(
                    point_motions[index], current_lidar_translation);
                if (!std::isfinite(origin[0]) || !std::isfinite(origin[1]) ||
                    !std::isfinite(origin[2]) ||
                    !std::isfinite(point_motions[index].time_from_scan_end_seconds)) continue;
                clearance_points.push_back({
                    point.x, point.y, point.z,
                    static_cast<float>(point_motions[index].time_from_scan_end_seconds),
                    static_cast<float>(origin[0]), static_cast<float>(origin[1]),
                    static_cast<float>(origin[2])});
            }

            if (!clearance_points.empty())
            {
                sensor_msgs::msg::PointCloud2 clearance_msg;
                clearance_msg.header.stamp = get_ros_time(lidar_end_time);
                clearance_msg.header.frame_id = "body";
                sensor_msgs::PointCloud2Modifier modifier(clearance_msg);
                modifier.setPointCloud2Fields(
                    7,
                    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "time_from_scan_end", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "ray_origin_x", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "ray_origin_y", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "ray_origin_z", 1, sensor_msgs::msg::PointField::FLOAT32);
                modifier.resize(clearance_points.size());
                sensor_msgs::PointCloud2Iterator<float> out_x(clearance_msg, "x");
                sensor_msgs::PointCloud2Iterator<float> out_y(clearance_msg, "y");
                sensor_msgs::PointCloud2Iterator<float> out_z(clearance_msg, "z");
                sensor_msgs::PointCloud2Iterator<float> out_time(
                    clearance_msg, "time_from_scan_end");
                sensor_msgs::PointCloud2Iterator<float> out_origin_x(
                    clearance_msg, "ray_origin_x");
                sensor_msgs::PointCloud2Iterator<float> out_origin_y(
                    clearance_msg, "ray_origin_y");
                sensor_msgs::PointCloud2Iterator<float> out_origin_z(
                    clearance_msg, "ray_origin_z");
                for (const auto & point : clearance_points)
                {
                    *out_x = point[0];
                    *out_y = point[1];
                    *out_z = point[2];
                    *out_time = point[3];
                    *out_origin_x = point[4];
                    *out_origin_y = point[5];
                    *out_origin_z = point[6];
                    ++out_x;
                    ++out_y;
                    ++out_z;
                    ++out_time;
                    ++out_origin_x;
                    ++out_origin_y;
                    ++out_origin_z;
                }
                pubLaserCloudClearance->publish(clearance_msg);
            }
        }
    }
    publish_count -= PUBFRAME_PERIOD;
}

void publish_frame_body_downsampled(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud_body_downsampled)
{
    if (pubLaserCloud_body_downsampled->get_subscription_count() == 0)
    {
        return;
    }

    int size = feats_down_body->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_down_body->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloud_body_downsampled->publish(laserCloudmsg);
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "odom";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

bool snapshot_map(PointCloudXYZI &map)
{
    if (ikdtree.Root_Node == nullptr)
    {
        return false;
    }

    PointVector().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
    map.clear();
    map.points = ikdtree.PCL_Storage;
    map.width = static_cast<std::uint32_t>(map.points.size());
    map.height = 1;
    return !map.empty();
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    if (pubLaserCloudMap->get_subscription_count() == 0)
    {
        return;
    }

    PointCloudXYZI map_snapshot;
    if (!snapshot_map(map_snapshot))
    {
        return;
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(map_snapshot, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    pubLaserCloudMap->publish(laserCloudmsg);
}

void save_to_pcd()
{
    if (complete_map->empty())
    {
        throw std::runtime_error("FAST-LIO complete map is empty");
    }

    pcl::PCDWriter pcd_writer;
    if (pcd_writer.writeBinary(map_file_path, *complete_map) != 0)
    {
        throw std::runtime_error("failed to write PCD: " + map_file_path);
    }
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br, bool zero_velocity = false)
{
    odomAftMapped.header.frame_id = "odom";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    const V3D velocity_body = state_point.rot.conjugate() * state_point.vel;
    odomAftMapped.twist.twist.linear.x = velocity_body(0);
    odomAftMapped.twist.twist.linear.y = velocity_body(1);
    odomAftMapped.twist.twist.linear.z = velocity_body(2);
    odomAftMapped.twist.twist.angular.x = 0.0;
    odomAftMapped.twist.twist.angular.y = 0.0;
    odomAftMapped.twist.twist.angular.z = 0.0;
    if (!Measures.imu.empty())
    {
        const auto &imu = Measures.imu.back();
        odomAftMapped.twist.twist.angular.x = imu->angular_velocity.x - state_point.bg(0);
        odomAftMapped.twist.twist.angular.y = imu->angular_velocity.y - state_point.bg(1);
        odomAftMapped.twist.twist.angular.z = imu->angular_velocity.z - state_point.bg(2);
    }
    if (zero_velocity)
    {
        odomAftMapped.twist.twist.linear.x = 0.0;
        odomAftMapped.twist.twist.linear.y = 0.0;
        odomAftMapped.twist.twist.linear.z = 0.0;
        odomAftMapped.twist.twist.angular.x = 0.0;
        odomAftMapped.twist.twist.angular.y = 0.0;
        odomAftMapped.twist.twist.angular.z = 0.0;
    }
    const Eigen::Matrix3d rotation = state_point.rot.toRotationMatrix();
    const Eigen::Matrix3d velocity_covariance =
        rotation.transpose() * P.block<3, 3>(12, 12) * rotation;
    const Eigen::Matrix3d angular_covariance = P.block<3, 3>(3, 3);
    odomAftMapped.twist.covariance.fill(0.0);
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            odomAftMapped.twist.covariance[row * 6 + col] = velocity_covariance(row, col);
            odomAftMapped.twist.covariance[(row + 3) * 6 + col + 3] = angular_covariance(row, col);
        }
    }

    pubOdomAftMapped->publish(odomAftMapped);

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "odom";
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = "body";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "odom";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    const double effective_ratio = feats_down_size > 0 ?
        static_cast<double>(effct_feat_num) / static_cast<double>(feats_down_size) : 0.0;
    res_mean_last = effct_feat_num > 0 ? total_residual / effct_feat_num :
        std::numeric_limits<double>::infinity();
    if (effct_feat_num < health_min_effective_points ||
        effective_ratio < health_min_effective_ratio ||
        !std::isfinite(res_mean_last) || res_mean_last > health_max_mean_residual)
    {
        ekfom_data.valid = false;
        /*** The solver may iterate several times over one scan; record the reason
             once so the frame still counts as a single fault. ***/
        if ((scan_fault_reasons & fast_lio::SCAN_FAULT_MEASUREMENT_GATE) == 0u)
        {
            scan_fault_reasons |= fast_lio::SCAN_FAULT_MEASUREMENT_GATE;
            std::string rejection_reasons;
            if (effct_feat_num < health_min_effective_points) {
                rejection_reasons += "effective_count";
            }
            if (effective_ratio < health_min_effective_ratio) {
                rejection_reasons += rejection_reasons.empty() ? "effective_ratio" : ",effective_ratio";
            }
            if (!std::isfinite(res_mean_last) || res_mean_last > health_max_mean_residual) {
                rejection_reasons += rejection_reasons.empty() ? "residual" : ",residual";
            }
            std::cerr << "FAST-LIO measurement gate rejected scan: reason="
                      << rejection_reasons << " scan_stamp=" << Measures.lidar_end_time
                      << " points_raw=" << current_scan_raw_point_count
                      << " points_preprocessed=" << current_scan_preprocessed_point_count
                      << " points_deskewed=" << feats_undistort->points.size()
                      << " points_voxelized=" << feats_down_size
                      << " points_effective=" << effct_feat_num
                      << " ratio=" << effective_ratio
                      << " residual=" << res_mean_last
                      << " limits=(effective>=" << health_min_effective_points
                      << ",ratio>=" << health_min_effective_ratio
                      << ",residual<=" << health_max_mean_residual << ")" << std::endl;
        }
        return;
    }

    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 1);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", false);
        this->declare_parameter<double>("mapping.body_to_base_roll", 0.0);
        this->declare_parameter<double>("mapping.body_to_base_pitch", 0.0);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<int>("health.min_effective_points", 10);
        this->declare_parameter<double>("health.min_effective_ratio", 0.02);
        this->declare_parameter<double>("health.max_mean_residual", 0.15);
        this->declare_parameter<double>("health.max_translation_correction", 0.30);
        this->declare_parameter<double>("health.max_rotation_correction", 0.35);
        this->declare_parameter<double>("health.max_speed", 1.5);
        this->declare_parameter<double>("health.max_vertical_speed", 0.5);
        this->declare_parameter<double>("health.max_imu_gap", 0.05);
        this->declare_parameter<double>("health.max_lidar_gap", 0.50);
        this->declare_parameter<double>("health.reinit_imu_gap", 0.30);
        this->declare_parameter<double>("health.reinit_lidar_gap", 1.00);
        this->declare_parameter<double>("health.max_processing_lag", 0.60);
        this->declare_parameter<int>("health.max_consecutive_faults", 10);
        this->declare_parameter<double>("health.min_consecutive_fault_duration", 1.0);
        this->declare_parameter<bool>("health.shutdown_on_fault", false);
        this->declare_parameter<bool>("health.shutdown_on_relocalization_lock", true);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner",filter_size_corner_min,0.5);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,200.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 1);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, false);
        double body_to_base_roll = 0.0;
        double body_to_base_pitch = 0.0;
        this->get_parameter_or<double>(
            "mapping.body_to_base_roll", body_to_base_roll, 0.0);
        this->get_parameter_or<double>(
            "mapping.body_to_base_pitch", body_to_base_pitch, 0.0);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<int>("health.min_effective_points", health_min_effective_points, 10);
        this->get_parameter_or<double>("health.min_effective_ratio", health_min_effective_ratio, 0.02);
        this->get_parameter_or<double>("health.max_mean_residual", health_max_mean_residual, 0.15);
        this->get_parameter_or<double>("health.max_translation_correction", health_max_translation_correction, 0.30);
        this->get_parameter_or<double>("health.max_rotation_correction", health_max_rotation_correction, 0.35);
        this->get_parameter_or<double>("health.max_speed", health_max_speed, 1.5);
        this->get_parameter_or<double>("health.max_vertical_speed", health_max_vertical_speed, 0.5);
        this->get_parameter_or<double>("health.max_imu_gap", health_max_imu_gap, 0.05);
        this->get_parameter_or<double>("health.max_lidar_gap", health_max_lidar_gap, 0.50);
        this->get_parameter_or<double>("health.reinit_imu_gap", health_reinit_imu_gap, 0.30);
        this->get_parameter_or<double>("health.reinit_lidar_gap", health_reinit_lidar_gap, 1.00);
        this->get_parameter_or<double>("health.max_processing_lag", health_max_processing_lag, 0.60);
        this->get_parameter_or<int>("health.max_consecutive_faults", health_max_consecutive_faults, 10);
        this->get_parameter_or<double>(
            "health.min_consecutive_fault_duration", health_min_consecutive_fault_duration, 1.0);
        this->get_parameter_or<bool>("health.shutdown_on_fault", health_shutdown_on_fault, false);
        this->get_parameter_or<bool>(
            "health.shutdown_on_relocalization_lock", health_shutdown_on_relocalization_lock, true);
        scan_health_monitor.set_max_consecutive_faults(health_max_consecutive_faults);
        scan_health_monitor.set_min_fault_duration(health_min_consecutive_fault_duration);
        // A normal-health gap is itself an abnormal frame. Only a full
        // reinitialization-sized silence breaks the fault run. No timer advances it.
        scan_health_monitor.set_max_inter_scan_gap(health_reinit_lidar_gap);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());

        RCLCPP_INFO(
            this->get_logger(),
            "FAST-LIO effective input config: lidar_type=%d point_filter_num=%d "
            "filter_size_surf=%.3f m min_effective_points=%d min_effective_ratio=%.4f "
            "max_mean_residual=%.4f fault_confirmation=%d scans/%.2f s",
            p_pre->lidar_type, p_pre->point_filter_num, filter_size_surf_min,
            health_min_effective_points, health_min_effective_ratio, health_max_mean_residual,
            health_max_consecutive_faults, health_min_consecutive_fault_duration);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id ="odom";

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        const M3D body_to_base_rotation =
            Eigen::AngleAxisd(body_to_base_pitch, V3D::UnitY()).toRotationMatrix() *
            Eigen::AngleAxisd(body_to_base_roll, V3D::UnitX()).toRotationMatrix();
        input_level_rotation = body_to_base_rotation.transpose();
        Lidar_T_wrt_IMU = input_level_rotation * Lidar_T_wrt_IMU;
        Lidar_R_wrt_IMU = input_level_rotation * Lidar_R_wrt_IMU;
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        raw_body_tf_broadcaster_ =
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        publish_raw_body_tf(this->get_clock()->now());
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(),"w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

        /*** ROS subscribe initialization ***/
        lidar_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        imu_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        processing_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions lidar_options;
        lidar_options.callback_group = lidar_callback_group_;
        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = imu_callback_group_;
        if (p_pre->lidar_type == AVIA)
        {
            // Reliable: the Livox driver publishes reliably, and a dropped scan is a
            // measurement gap the health gate has to treat as a fault.
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lid_topic, rclcpp::QoS(5), livox_pcl_cbk, lidar_options);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, rclcpp::SensorDataQoS().keep_last(5), standard_pcl_cbk, lidar_options);
        }
        // Reliable: IMU samples drive state propagation, so a lost sample is an
        // unrecoverable hole rather than a stale reading.
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::QoS(rclcpp::KeepLast(MAX_IMU_BUFFER_SIZE)), imu_cbk, imu_options);
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudClearance_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered_body_clearance", 20);
        pubLaserCloudBodyDownsampled_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body_downsampled", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pub_fastlio_health = this->create_publisher<std_msgs::msg::Bool>(
            "/fastlio/health", rclcpp::QoS(1));
        pub_fastlio_recovery_state = this->create_publisher<std_msgs::msg::String>(
            "/fastlio/recovery_state", rclcpp::QoS(1).transient_local());
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/Laser_map", rclcpp::SensorDataQoS().keep_last(1));
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        timer_ = this->create_wall_timer(
            period_ms, std::bind(&LaserMappingNode::timer_callback, this), processing_callback_group_);

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = this->create_wall_timer(
            map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this), processing_callback_group_);

        // Same mutually-exclusive group as the processing timers: the callback walks
        // the same complete-map buffer the scan pipeline appends to.
        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "map_save",
            std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            processing_callback_group_);

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        fout_out.close();
        fout_pre.close();
        if (fp != nullptr)
        {
            fclose(fp);
        }
    }

private:
    void restore_last_good_state()
    {
        if (!has_last_good_state) return;
        last_good_state.vel.setZero();
        last_good_covariance.block<3, 3>(12, 12).diagonal() =
            last_good_covariance.block<3, 3>(12, 12).diagonal().cwiseMax(0.25);
        kf.change_x(last_good_state);
        kf.change_P(last_good_covariance);
        state_point = last_good_state;
        refresh_output_state();
    }

    void reinitialize_odometry(bool restore_state)
    {
        {
            std::lock_guard<std::mutex> lock(mtx_buffer);
            lidar_queue.clear();
            imu_buffer.clear();
            is_first_lidar = true;
            last_timestamp_lidar = 0.0;
            last_timestamp_imu = -1.0;
            previous_imu_timestamp = -1.0;
            timediff_set_flg = false;
            timediff_lidar_wrt_imu = 0.0;
            last_imu_receive_time = {};
            last_lidar_receive_time = {};
        }
        imu_gap_detected.store(false);
        lidar_queue_drop_detected.store(false);
        imu_reinit_gap_detected.store(false);
        lidar_reinit_gap_detected.store(false);

        const bool preserve_calibration = restore_state && has_last_good_state;
        prime_imu_time_axis_after_reset = preserve_calibration;
        if (preserve_calibration)
        {
            last_good_state.vel.setZero();
            if (!recovery_required)
            {
                last_good_covariance.block<3, 3>(12, 12).diagonal() =
                    last_good_covariance.block<3, 3>(12, 12).diagonal().cwiseMax(0.25);
            }
            restore_last_good_state();
        }

        p_imu->Reset(preserve_calibration);
        feats_undistort->clear();
        flg_first_scan = true;
        flg_EKF_inited = false;
        lidar_mean_scantime = 0.0;
        scan_num = 0;
        previous_processed_lidar_timestamp = -1.0;
        scan_fault_reasons = fast_lio::SCAN_FAULT_NONE;
        have_current_scan_prediction = false;
        scan_health_monitor.reset();
        RCLCPP_WARN(
            this->get_logger(),
            "FAST-LIO IMU pipeline reset; preserving the last trusted odometry pose");
    }

    void hold_last_good_pose(
        const char *reason, bool publish_pose = true, bool preserve_candidate = true)
    {
        const bool can_preserve_candidate = preserve_candidate && has_last_good_state &&
            ekf_state_is_finite(state_point) && kf.get_P().array().isFinite().all();
        state_ikfom candidate_state;
        esekfom::esekf<state_ikfom, 12, input_ikfom>::cov candidate_covariance;
        if (can_preserve_candidate)
        {
            candidate_state = state_point;
            candidate_covariance = kf.get_P();
        }
        restore_last_good_state();
        publish_fastlio_health(false);
        if (!recovery_required && !recovery_requires_relocalization)
        {
            publish_fastlio_recovery_state("DEGRADED");
        }
        if (has_last_good_state && publish_pose) {
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_, true);
        }
        if (can_preserve_candidate)
        {
            kf.change_x(candidate_state);
            kf.change_P(candidate_covariance);
            state_point = candidate_state;
            refresh_output_state();
        }
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "FAST-LIO holding last trusted pose: %s", reason);
    }

    void begin_recovery(
        bool require_relocalization, const std::string &trigger_reason,
        double translation_error = std::numeric_limits<double>::quiet_NaN(),
        double rotation_error = std::numeric_limits<double>::quiet_NaN())
    {
        // Runtime GICP is not kept alive by the map-navigation launch. Once the
        // trusted pose window is exceeded (or the time axis becomes fatal), keep
        // this node latched unhealthy until the full localization stack restarts.
        if (recovery_requires_relocalization && !require_relocalization) {
            return;
        }
        const bool starting_recovery = !recovery_required && has_last_good_state;
        recovery_required = has_last_good_state;
        recovery_requires_relocalization = require_relocalization && has_last_good_state;
        if (starting_recovery)
        {
            last_good_state.vel.setZero();
            last_good_covariance.block<3, 3>(12, 12).diagonal() =
                last_good_covariance.block<3, 3>(12, 12).diagonal().cwiseMax(0.25);
        }
        if (recovery_required && !recovery_requires_relocalization) {
            odometry_recovery_gate.start();
            publish_fastlio_recovery_state("RECOVERING");
            RCLCPP_WARN(
                this->get_logger(),
                "FAST-LIO recovery started: mode=trusted_odometry trigger=%s "
                "hold_last_good_pose=true",
                trigger_reason.c_str());
        } else {
            odometry_recovery_gate.reset();
            if (recovery_requires_relocalization)
            {
                publish_fastlio_recovery_state("RELOCALIZATION_REQUIRED");
                RCLCPP_FATAL(
                    this->get_logger(),
                    "FAST-LIO recovery escalation: category=relocalization_required "
                    "trigger=%s pose_error_m=%.6f pose_error_rad=%.6f "
                    "trusted_limits_m=0.100000 trusted_limits_rad=0.050000 "
                    "restart_full_localization=true",
                    trigger_reason.c_str(), translation_error, rotation_error);
                if (health_shutdown_on_relocalization_lock && rclcpp::ok())
                {
                    rclcpp::shutdown();
                }
            }
        }
    }

    /*** Publish only trusted poses. A runtime fault holds the last accepted state until
         a bounded, consecutive recovery sequence proves that FAST-LIO is aligned again. ***/
    bool finish_frame(std::uint32_t reasons)
    {
        double prediction_translation_error = 0.0;
        double prediction_rotation_error = 0.0;
        if ((reasons & ~fast_lio::SCAN_FAULT_LIDAR_QUEUE_DROP) ==
                fast_lio::SCAN_FAULT_NONE && have_current_scan_prediction)
        {
            const Eigen::Matrix3d delta_rotation =
                current_scan_prediction.rot.toRotationMatrix().transpose() *
                state_point.rot.toRotationMatrix();
            prediction_translation_error =
                (state_point.pos - current_scan_prediction.pos).norm();
            prediction_rotation_error = Eigen::AngleAxisd(delta_rotation).angle();
            if (!std::isfinite(prediction_translation_error) ||
                !std::isfinite(prediction_rotation_error))
            {
                reasons |= fast_lio::SCAN_FAULT_NON_FINITE_STATE;
            }
            else if (prediction_translation_error > 0.10 || prediction_rotation_error > 0.05)
            {
                reasons |= fast_lio::SCAN_FAULT_IMPLAUSIBLE_UPDATE;
                kf.change_x(current_scan_prediction);
                kf.change_P(current_scan_prediction_covariance);
                state_point = current_scan_prediction;
                refresh_output_state();
            }
        }

        const std::string reason_text = fast_lio::scan_fault_reason_text(reasons);
        const auto state_fault_reasons = reasons & ~fast_lio::SCAN_FAULT_LIDAR_QUEUE_DROP;
        if (!has_last_good_state && state_fault_reasons != fast_lio::SCAN_FAULT_NONE)
        {
            // Reject state faults until a clean scan establishes the initial trusted pose.
            const auto initial_decision =
                scan_health_monitor.evaluate(reasons, Measures.lidar_beg_time);
            publish_fastlio_health(false);
            if ((reasons & fast_lio::SCAN_FAULT_FATAL_MASK) != 0u) {
                reinitialize_odometry(false);
            } else if (initial_decision.outcome ==
                fast_lio::ScanOutcome::RELOCALIZATION_REQUIRED)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "FAST-LIO initial scan fault run confirmed; restarting scan initialization: "
                    "frames=%d duration=%.3f s",
                    initial_decision.consecutive_faults, initial_decision.fault_duration_sec);
                reinitialize_odometry(false);
            }
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "FAST-LIO waiting for initial trusted scan: %s", reason_text.c_str());
            return false;
        }

        const auto decision = scan_health_monitor.evaluate(reasons, Measures.lidar_beg_time);
        if (decision.outcome == fast_lio::ScanOutcome::DUPLICATE)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "FAST-LIO discarded duplicate or old scan: stamp=%.9f",
                Measures.lidar_beg_time);
            return false;
        }
        if (decision.outcome != fast_lio::ScanOutcome::ACCEPTED)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "FAST-LIO rejected scan stages: reason=%s raw=%zu preprocessed=%zu "
                "deskewed=%zu voxelized=%d effective=%d ratio=%.6f residual=%.6f",
                reason_text.c_str(), current_scan_raw_point_count,
                current_scan_preprocessed_point_count, feats_undistort->points.size(),
                feats_down_size, effct_feat_num,
                feats_down_size > 0 ? static_cast<double>(effct_feat_num) / feats_down_size : 0.0,
                res_mean_last);
            if (decision.outcome == fast_lio::ScanOutcome::FATAL)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "FAST-LIO health escalation: outcome=fatal reason=%s consecutive=%d "
                    "max_consecutive_faults=%d hard_fault=%s",
                    reason_text.c_str(), decision.consecutive_faults,
                    health_max_consecutive_faults, decision.hard_fault ? "true" : "false");
                begin_recovery(true, "fatal_scan_health:" + reason_text);
                scan_health_monitor.reset();
                const bool time_regression =
                    (reasons & fast_lio::SCAN_FAULT_TIME_REGRESSION) != 0u;
                hold_last_good_pose(reason_text.c_str(), !time_regression, false);
                reinitialize_odometry(true);
                recovery_required = has_last_good_state;
                recovery_requires_relocalization = has_last_good_state;
                publish_fastlio_recovery_state(
                    recovery_requires_relocalization ? "RELOCALIZATION_REQUIRED" : "RECOVERING");
                return false;
            }
            if (have_current_scan_prediction)
            {
                // Continue from motion prediction, never a rejected scan correction.
                kf.change_x(current_scan_prediction);
                kf.change_P(current_scan_prediction_covariance);
                state_point = current_scan_prediction;
                refresh_output_state();
            }
            if (decision.outcome == fast_lio::ScanOutcome::RELOCALIZATION_REQUIRED) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "FAST-LIO health escalation: outcome=relocalization_required reason=%s "
                    "consecutive=%d duration=%.3f s "
                    "max_consecutive_faults=%d points=(raw %zu preprocessed %zu deskewed %zu "
                    "voxelized %d effective %d) ratio=%.6f residual=%.6f",
                    reason_text.c_str(), decision.consecutive_faults,
                    decision.fault_duration_sec,
                    health_max_consecutive_faults, current_scan_raw_point_count,
                    current_scan_preprocessed_point_count, feats_undistort->points.size(),
                    feats_down_size, effct_feat_num,
                    feats_down_size > 0 ? static_cast<double>(effct_feat_num) / feats_down_size : 0.0,
                    res_mean_last);
                begin_recovery(true, "confirmed_scan_faults:" + reason_text);
                scan_health_monitor.reset();
            } else if (!recovery_required) {
                begin_recovery(false, "single_or_recoverable_scan_fault:" + reason_text);
            }
            if (recovery_required && !recovery_requires_relocalization) {
                odometry_recovery_gate.observe(false, 0.0, 0.0);
            }

            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "FAST-LIO rejected scan (%s); holding trusted pose (%d consecutive)",
                reason_text.c_str(), decision.consecutive_faults);
            hold_last_good_pose(
                reason_text.c_str(), true,
                decision.outcome != fast_lio::ScanOutcome::RELOCALIZATION_REQUIRED);
            if (decision.outcome == fast_lio::ScanOutcome::RELOCALIZATION_REQUIRED &&
                health_shutdown_on_fault) {
                RCLCPP_ERROR(this->get_logger(), "FAST-LIO hard health fault; stopping stack");
                if (rclcpp::ok()) rclcpp::shutdown();
            }
            return false;
        }

        if (recovery_requires_relocalization) {
            hold_last_good_pose("a new GICP initial pose is required", true, false);
            return false;
        }
        if (recovery_required) {
            const auto recovery_result = odometry_recovery_gate.observe_candidate(
                true, prediction_translation_error, prediction_rotation_error,
                Measures.lidar_beg_time);
            if (recovery_result == fast_lio::RecoveryGate::Result::WAITING) {
                hold_last_good_pose(
                    "waiting for five trusted scans over 0.5 seconds", true, true);
                return false;
            }
            recovery_required = false;
            odometry_recovery_gate.reset();
            publish_fastlio_recovery_state("NORMAL");
            RCLCPP_INFO(this->get_logger(),
                "FAST-LIO odometry recovered after stable scans within the trusted pose window");
        }

        publish_fastlio_health(true);
        if (!recovery_required && !recovery_requires_relocalization)
        {
            publish_fastlio_recovery_state("NORMAL");
        }
        last_good_state = state_point;
        last_good_covariance = kf.get_P();
        has_last_good_state = true;
        return true;
    }

    void timer_callback()
    {
        /*** A sensor stamp jumping backwards invalidates the filter's time axis, so it
             cannot be counted away; handle it before touching any measurement. ***/
        if (sensor_time_regression.exchange(false))
        {
            finish_frame(fast_lio::SCAN_FAULT_TIME_REGRESSION);
            return;
        }

        if(sync_packages(Measures))
        {
            effct_feat_num = 0;
            feats_down_size = 0;
            have_current_scan_prediction = false;
            if (!std::isfinite(Measures.lidar_beg_time) ||
                !std::isfinite(Measures.lidar_end_time))
            {
                finish_frame(fast_lio::SCAN_FAULT_TIME_REGRESSION);
                return;
            }
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                previous_processed_lidar_timestamp = Measures.lidar_beg_time;
                imu_gap_detected.store(false);
                lidar_queue_drop_detected.store(false);
                if (prime_imu_time_axis_after_reset && Measures.imu.empty())
                {
                    publish_fastlio_health(false);
                    return;
                }
                const bool publish_recovery_hold = prime_imu_time_axis_after_reset;
                if (publish_recovery_hold)
                {
                    p_imu->PrimeTimeAxis(Measures, state_point);
                    prime_imu_time_axis_after_reset = false;
                }
                flg_first_scan = false;
                if (publish_recovery_hold) hold_last_good_pose("fresh time axis");
                return;
            }

            if (Measures.lidar_beg_time <= previous_processed_lidar_timestamp)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "FAST-LIO discarded duplicate or old scan before propagation: "
                    "stamp=%.9f previous=%.9f",
                    Measures.lidar_beg_time, previous_processed_lidar_timestamp);
                return;
            }

            /*** Input-health gates. Every reason is collected into scan_fault_reasons
                 and counted once at the end of the frame. Lost samples make the scan
                 untrustworthy for the map; late-but-complete data does not. ***/
            scan_fault_reasons = fast_lio::SCAN_FAULT_NONE;
            const double lidar_gap = Measures.lidar_beg_time - previous_processed_lidar_timestamp;
            previous_processed_lidar_timestamp = Measures.lidar_beg_time;
            last_lidar_gap.store(lidar_gap);
            if (fast_lio::lidar_gap_exceeds_threshold(lidar_gap, health_max_lidar_gap))
            {
                scan_fault_reasons |= fast_lio::SCAN_FAULT_LIDAR_GAP;
            }
            if (lidar_queue_drop_detected.exchange(false))
            {
                scan_fault_reasons |= fast_lio::SCAN_FAULT_LIDAR_QUEUE_DROP;
            }
            if (imu_gap_detected.exchange(false) || Measures.imu.empty())
            {
                scan_fault_reasons |= fast_lio::SCAN_FAULT_IMU_GAP;
            }

            /*** Compute backlog is late data, not lost data: warn, but leave the scan
                 eligible for the map so a busy board does not stop the chassis. ***/
            const double processing_lag = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - current_scan_received_at).count();
            if (processing_lag > health_max_processing_lag)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "FAST-LIO processing backlog: lag=%.3f s", processing_lag);
            }
            if (scan_fault_reasons != fast_lio::SCAN_FAULT_NONE)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "FAST-LIO input status (%s): header_gap=(lidar %.3f imu %.3f)s "
                    "receive_gap=(lidar %.3f imu %.3f)s processing_lag=%.3fs "
                    "queue_drops=(lidar %llu imu %llu)",
                    fast_lio::scan_fault_reason_text(scan_fault_reasons).c_str(),
                    last_lidar_gap.load(), last_imu_gap.load(),
                    last_lidar_receive_gap.load(), last_imu_receive_gap.load(), processing_lag,
                    static_cast<unsigned long long>(dropped_lidar_scans.load()),
                    static_cast<unsigned long long>(dropped_imu_samples.load()));
            }

            const std::uint32_t gap_faults = fast_lio::SCAN_FAULT_LIDAR_GAP |
                fast_lio::SCAN_FAULT_IMU_GAP;
            if ((scan_fault_reasons & gap_faults) != 0u) {
                RCLCPP_WARN(this->get_logger(),
                    "FAST-LIO discarded scan before IMU propagation after sensor gap");
                const bool long_gap = imu_reinit_gap_detected.exchange(false) ||
                    lidar_reinit_gap_detected.exchange(false);
                finish_frame(scan_fault_reasons);
                if (long_gap && !recovery_requires_relocalization) {
                    reinitialize_odometry(has_last_good_state);
                    recovery_required = has_last_good_state;
                    recovery_requires_relocalization = false;
                }
                return;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            /*** ImuProcess leaves its output cloud untouched when it returns early
                 (still initialising, or no IMU in this window). Clear it first so such
                 a frame reports an empty scan instead of reusing the previous one. ***/
            feats_undistort->clear();
            const bool imu_initializing_before = p_imu->is_initializing();
            p_imu->Process(Measures, kf, feats_undistort);
            if (imu_initializing_before || p_imu->is_initializing()) {
                if (has_last_good_state) {
                    hold_last_good_pose("waiting for IMU initialization after sensor reset");
                } else {
                    publish_fastlio_health(false);
                }
                return;
            }
            state_ikfom predicted_state = kf.get_x();
            auto predicted_covariance = kf.get_P();
            current_scan_prediction = predicted_state;
            current_scan_prediction_covariance = predicted_covariance;
            have_current_scan_prediction = true;
            state_point = kf.get_x();
            refresh_output_state();

            /*** Nothing downstream can be trusted once propagation itself diverges, and
                 no later gate can bring it back. ***/
            if (!ekf_state_is_finite(state_point) || !predicted_covariance.array().isFinite().all())
            {
                finish_frame(scan_fault_reasons | fast_lio::SCAN_FAULT_NON_FINITE_STATE);
                return;
            }

            if (extrinsic_est_en)
            {
                const M3D extrinsic_R = state_point.offset_R_L_I.toRotationMatrix();
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Estimated LiDAR extrinsic R=[%.5f %.5f %.5f; %.5f %.5f %.5f; %.5f %.5f %.5f], T=[%.5f %.5f %.5f] m",
                    extrinsic_R(0, 0), extrinsic_R(0, 1), extrinsic_R(0, 2),
                    extrinsic_R(1, 0), extrinsic_R(1, 1), extrinsic_R(1, 2),
                    extrinsic_R(2, 0), extrinsic_R(2, 1), extrinsic_R(2, 2),
                    state_point.offset_T_L_I.x(), state_point.offset_T_L_I.y(), state_point.offset_T_L_I.z());
            }

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "FAST-LIO scan stage rejected: reason=empty_after_deskew "
                    "scan_stamp=%.9f points_raw=%zu points_preprocessed=%zu "
                    "points_deskewed=0 points_voxelized=0 points_effective=0",
                    Measures.lidar_end_time, current_scan_raw_point_count,
                    current_scan_preprocessed_point_count);
                finish_frame(scan_fault_reasons | fast_lio::SCAN_FAULT_EMPTY_SCAN);
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                finish_frame(scan_fault_reasons | fast_lio::SCAN_FAULT_MAP_NOT_READY);
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "FAST-LIO scan stage rejected: reason=voxelized_below_solver_minimum "
                    "scan_stamp=%.9f points_raw=%zu points_preprocessed=%zu "
                    "points_deskewed=%zu points_voxelized=%d points_effective=0",
                    Measures.lidar_end_time, current_scan_raw_point_count,
                    current_scan_preprocessed_point_count, feats_undistort->points.size(),
                    feats_down_size);
                finish_frame(scan_fault_reasons | fast_lio::SCAN_FAULT_EMPTY_SCAN);
                return;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            const bool finite_state = ekf_state_is_finite(state_point);
            const V3D translation_correction = state_point.pos - predicted_state.pos;
            const Eigen::Matrix3d delta_rotation =
                predicted_state.rot.toRotationMatrix().transpose() *
                state_point.rot.toRotationMatrix();
            const double rotation_correction =
                Eigen::AngleAxisd(delta_rotation).angle();
            const auto updated_covariance = kf.get_P();
            const bool finite_estimate =
                finite_state && updated_covariance.array().isFinite().all();
            const bool plausible_correction =
                translation_correction.norm() <= health_max_translation_correction &&
                rotation_correction <= health_max_rotation_correction &&
                state_point.vel.norm() <= health_max_speed &&
                std::abs(state_point.vel.z()) <= health_max_vertical_speed;
            if (!finite_estimate || !plausible_correction)
            {
                const double rejected_speed = state_point.vel.norm();
                const double rejected_vertical_speed = std::abs(state_point.vel.z());
                /*** Roll back to the propagated state: it is the last estimate that is
                     still trustworthy, and it stays finite even when the update is not. ***/
                kf.change_x(predicted_state);
                kf.change_P(predicted_covariance);
                state_point = predicted_state;
                scan_fault_reasons |= finite_estimate
                    ? fast_lio::SCAN_FAULT_IMPLAUSIBLE_UPDATE
                    : fast_lio::SCAN_FAULT_NON_FINITE_STATE;
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Rejecting FAST-LIO update: finite=%d residual=%.3f dpos=%.3f drot=%.3f speed=%.3f vz=%.3f",
                    finite_estimate, res_mean_last, translation_correction.norm(),
                    rotation_correction, rejected_speed, rejected_vertical_speed);
            }
            /*** Refresh the published pose from whichever state survived the gate, so
                 position and orientation always come from the same frame. ***/
            refresh_output_state();

            double t_update_end = omp_get_wtime();

            /*** Single exit point for the frame verdict: it counts the frame once,
                 publishes health, and on a rejected frame publishes the propagated
                 odometry before returning without touching the map. ***/
            if (!finish_frame(scan_fault_reasons))
            {
                return;
            }

            if (pcd_save_en)
            {
                append_complete_scan_to_map();
            }

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath_);
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull_);
            if (scan_body_pub_en)
            {
                publish_frame_body(pubLaserCloudFull_body_, pubLaserCloudClearance_);
                publish_frame_body_downsampled(pubLaserCloudBodyDownsampled_);
            }
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                if (fp != nullptr)
                {
                    dump_lio_state_to_log(fp);
                }
            }
        }
    }

    void map_publish_callback()
    {
        if (map_pub_en) publish_map(pubLaserCloudMap_);
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en && !map_file_path.empty())
        {
            try
            {
                save_to_pcd();
                res->success = true;
                res->message = "Map saved.";
            }
            catch (const std::exception &error)
            {
                res->success = false;
                res->message = error.what();
            }
        }
        else
        {
            res->success = false;
            res->message = pcd_save_en ? "Map path is empty." : "Map save disabled.";
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudClearance_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudBodyDownsampled_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
    rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
    rclcpp::CallbackGroup::SharedPtr processing_callback_group_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    auto node = std::make_shared<LaserMappingNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();

    if (rclcpp::ok())
        rclcpp::shutdown();
    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        if (fp2 == nullptr)
        {
            std::cerr << "failed to open " << log_dir << std::endl;
            return 1;
        }
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
