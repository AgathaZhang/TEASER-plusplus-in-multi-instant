#include "lidar_compound.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

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
