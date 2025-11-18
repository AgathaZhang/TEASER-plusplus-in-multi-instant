#ifndef LIVOX_LONG_EXPOSURE_BUILDER_H
#define LIVOX_LONG_EXPOSURE_BUILDER_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LivoxLongExposureBuilder {
public:
  using PointT      = pcl::PointXYZI;
  using PointCloudT = pcl::PointCloud<PointT>;

  /**
   * @brief 构造函数
   * @param nh         ROS NodeHandle
   * @param topic_name 订阅的雷达话题名，默认 "/livox/lidar"
   */
  LivoxLongExposureBuilder(ros::NodeHandle& nh,
                           const std::string& topic_name = "/livox/lidar");

  /**
   * @brief 叠加合成长曝光点云（阻塞式）
   *
   * 调用后：
   *  - 清空内部累计点云
   *  - 在 duration_sec 时间内持续接收 /livox/lidar
   *  - 把收到的所有帧叠加到同一云中
   *  - 返回合成后的 PCL 点云指针
   *
   * @param duration_sec 长曝光时间（秒）
   * @return 合成后的点云指针
   */
  PointCloudT::Ptr buildLongExposureCloud(double duration_sec);

  /**
   * @brief 将当前合成点云导出为 PCD 文件
   *
   * @param output_path 输出路径（包含文件名，.pcd）
   * @return true 保存成功，false 失败或点云为空
   */
  bool saveCloudToPCD(const std::string& output_path) const;

private:
  /// ROS 点云回调：把每一帧转换为 PCL 并累加
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
  ros::Subscriber sub_;

  /// 内部累计的长曝光点云
  PointCloudT::Ptr accumulated_cloud_;
};

#endif  // LIVOX_LONG_EXPOSURE_BUILDER_H
