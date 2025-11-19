#pragma once
#ifndef LIDAR_FRAME_SYN_HPP_
#define LIDAR_FRAME_SYN_HPP_

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief 离线 rosbag 点云叠加类
 *
 * 功能：
 *  1. 从 rosbag 指定话题读取点云（默认 /livox/lidar）
 *  2. 在最多 duration_sec 时间窗口内叠加点云（默认 5s）
 *  3. 若 bag 总时长 < duration_sec，则叠加 bag 全部数据
 *  4. 可以将合成后的点云导出为 PCD 文件
 */
class LidarFrameSyn {
public:
  using PointT      = pcl::PointXYZI;
  using PointCloudT = pcl::PointCloud<PointT>;

  /**
   * @param topic_name rosbag 中要读取的点云话题名，默认 "/livox/lidar"
   */
  explicit LidarFrameSyn(const std::string& topic_name = "/livox/lidar");

  /**
   * @brief 从 rosbag 构建长曝光点云
   *
   * @param bag_path     rosbag 文件路径
   * @param duration_sec 最长叠加时间窗口（秒），默认 5.0
   *                     - 若 duration_sec <= 0，则叠加整个 bag
   *                     - 若 bag 内实际时长 < duration_sec，则叠加所有消息
   *
   * @return 合成后的点云指针（内部成员，勿删除，只读或拷贝）
   */
  PointCloudT::Ptr buildLongExposureCloudFromBag(const std::string& bag_path,
                                                 double duration_sec = 5.0);

  /**
   * @brief 将当前合成点云导出为 PCD 文件
   *
   * @param output_path 输出路径（包含文件名，.pcd）
   * @return true 保存成功，false 点云为空或保存失败
   */
  bool saveCloudToPCD(const std::string& output_path) const;

private:
  std::string topic_name_;
  PointCloudT::Ptr accumulated_cloud_;
};

#endif  // LIDAR_FRAME_SYN_HPP_
