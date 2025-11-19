#include "lidar_frame_syn.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

LidarFrameSyn::LidarFrameSyn(const std::string& topic_name)
  : topic_name_(topic_name),
    accumulated_cloud_(new PointCloudT) {}


LidarFrameSyn::PointCloudT::Ptr
LidarFrameSyn::buildLongExposureCloudFromBag(const std::string& bag_path,
                                             double duration_sec) {
  accumulated_cloud_->clear();

  try {
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(topic_name_);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    if (view.size() == 0) {
      ROS_WARN_STREAM("LidarFrameSyn: no messages found on topic "
                      << topic_name_ << " in bag: " << bag_path);
      bag.close();
      return accumulated_cloud_;
    }

    bool first_msg = true;
    ros::Time start_time;
    const bool limit_by_duration = (duration_sec > 0.0);

    ROS_INFO_STREAM("LidarFrameSyn: start accumulating from bag: "
                    << bag_path << ", topic: " << topic_name_
                    << ", duration_sec: " << duration_sec);

    for (const rosbag::MessageInstance& m : view) {
      if (first_msg) {
        start_time = m.getTime();
        first_msg = false;
      }

      if (limit_by_duration) {
        double elapsed = (m.getTime() - start_time).toSec();
        if (elapsed > duration_sec) {
          break;
        }
      }

      sensor_msgs::PointCloud2::ConstPtr msg =
          m.instantiate<sensor_msgs::PointCloud2>();
      if (!msg) {
        continue;
      }

      PointCloudT tmp_cloud;
      pcl::fromROSMsg(*msg, tmp_cloud);

      *accumulated_cloud_ += tmp_cloud;
    }

    bag.close();

    ROS_INFO_STREAM("LidarFrameSyn: accumulation finished. Total points: "
                    << accumulated_cloud_->size());
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("LidarFrameSyn: exception while reading bag: "
                     << e.what());
  }

  return accumulated_cloud_;
}

bool LidarFrameSyn::saveCloudToPCD(const std::string& output_path) const {
  if (!accumulated_cloud_ || accumulated_cloud_->empty()) {
    ROS_WARN_STREAM("LidarFrameSyn::saveCloudToPCD: accumulated cloud is empty, "
                    "nothing to save.");
    return false;
  }

  int ret = pcl::io::savePCDFileBinary(output_path, *accumulated_cloud_);
  if (ret != 0) {
    ROS_ERROR_STREAM("LidarFrameSyn::saveCloudToPCD: failed to save PCD to "
                     << output_path << ", error code = " << ret);
    return false;
  }

  ROS_INFO_STREAM("LidarFrameSyn::saveCloudToPCD: saved PCD to "
                  << output_path << " with "
                  << accumulated_cloud_->size() << " points.");
  return true;
}
