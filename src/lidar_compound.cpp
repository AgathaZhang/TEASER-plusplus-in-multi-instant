#include "lidar_compound.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

// 输入：4x4 齐次矩阵 T_bg（云台相对底盘）
// 输出：yaw, pitch（单位：弧度）
std::pair<double, double> decomposeYawPitch(const Eigen::Matrix3d& R)
{
  double yaw, pitch;

  // 1. 取 3x3 旋转部分
  // Eigen::Matrix3d R = T_bg.block<3,3>(0, 0);

  // 2. 根据 R ≈ Rz(yaw) * Ry(pitch) 近似反解 yaw 和 pitch
  //
  // 推导结果：
  // R(2,0) = -sin(pitch)
  // R(2,2) =  cos(pitch)
  // => pitch = atan2(-R(2,0), R(2,2))
  //
  // R(0,0) = cos(pitch) * cos(yaw)
  // R(1,0) = cos(pitch) * sin(yaw)
  // => yaw = atan2(R(1,0), R(0,0))
  //
  pitch = std::atan2(-R(2,0), R(2,2));
  yaw   = std::atan2( R(1,0), R(0,0));
  double yaw_deg   = yaw   * 180.0 / M_PI;
  double pitch_deg = pitch * 180.0 / M_PI;
  return {yaw_deg, pitch_deg};
}

LivoxLongExposureBuilder::LivoxLongExposureBuilder(ros::NodeHandle& nh,
                                                   const std::string& topic_name)
  : accumulated_cloud_(new PointCloudT) {
  // 订阅 Livox 雷达点云话题
  sub_ = nh.subscribe(topic_name,
                      10,  // 队列长度
                      &LivoxLongExposureBuilder::pointCloudCallback,
                      this);
}

void LivoxLongExposureBuilder::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  // 将 ROS 点云消息转换为 PCL 点云
  PointCloudT tmp_cloud;
  pcl::fromROSMsg(*msg, tmp_cloud);

  // 累加进长曝光点云
  *accumulated_cloud_ += tmp_cloud;
}

LivoxLongExposureBuilder::PointCloudT::Ptr
LivoxLongExposureBuilder::buildLongExposureCloud(double duration_sec) {
  if (duration_sec <= 0.0) {
    ROS_WARN_STREAM("LivoxLongExposureBuilder::buildLongExposureCloud: "
                    "duration_sec <= 0, nothing will be accumulated.");
    accumulated_cloud_->clear();
    return accumulated_cloud_;
  }

  // 清空历史累积
  accumulated_cloud_->clear();

  ros::Time start_time = ros::Time::now();
  ros::Rate rate(100.0);  // 控制 spinOnce 频率

  ROS_INFO_STREAM("LivoxLongExposureBuilder: start accumulating for "
                  << duration_sec << " seconds.");

  while (ros::ok()) {
    ros::spinOnce();

    ros::Duration elapsed = ros::Time::now() - start_time;
    if (elapsed.toSec() >= duration_sec) {
      break;
    }

    rate.sleep();
  }

  ROS_INFO_STREAM("LivoxLongExposureBuilder: accumulation finished. "
                  << "Total points: " << accumulated_cloud_->size());

  return accumulated_cloud_;
}

bool LivoxLongExposureBuilder::saveCloudToPCD(
    const std::string& output_path) const {
  if (!accumulated_cloud_ || accumulated_cloud_->empty()) {
    ROS_WARN_STREAM("LivoxLongExposureBuilder::saveCloudToPCD: "
                    "accumulated cloud is empty, nothing to save.");
    return false;
  }

  int ret = pcl::io::savePCDFileBinary(output_path, *accumulated_cloud_);
  if (ret != 0) {
    ROS_ERROR_STREAM("LivoxLongExposureBuilder::saveCloudToPCD: failed to save PCD to "
                     << output_path << ", error code = " << ret);
    return false;
  }

  ROS_INFO_STREAM("LivoxLongExposureBuilder::saveCloudToPCD: saved PCD to "
                  << output_path
                  << " with " << accumulated_cloud_->size() << " points.");
  return true;
}
