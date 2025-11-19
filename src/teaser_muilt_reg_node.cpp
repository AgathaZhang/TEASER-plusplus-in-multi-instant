#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/PCLPointCloud2.h> // agatha add
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h> 
#include <pcl/conversions.h>  
#include <pcl/common/io.h> 
#include <pcl/conversions.h>
#include <map>
#include <limits>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include "teaser_cpp_fpfh.hpp"

#include "lidar_compound.hpp"  // agatha add
#include "lidar_frame_syn.hpp"

// ====== 你项目里已有的函数：仅做前向声明 ======
// Eigen::Matrix4d map_icp_teaser_icp(
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_source,
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_target,
//     float scale);

// 让 test_match2 接口与调用处一致：返回 4x4，第三个参数当初是 Eigen::Affine3f 初始变换
// Eigen::Matrix4f test_match2(
//     const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_ds,
//     const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt_ds,
//     const Eigen::Affine3f& init_guess);

// downsample.hpp
#pragma once
#include <map>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace teaser_agatha {
  namespace ds {

  inline int signf(float v) { return (v > 0) - (v < 0); }

  template <typename T>
  pcl::PointCloud<T> downsample(typename pcl::PointCloud<T>::ConstPtr cloud,
                                float leaf_size,
                                float resolution = 10.0f) {
    pcl::VoxelGrid<T> voxel;
    typename pcl::PointCloud<T>::Ptr pc_rigid(new pcl::PointCloud<T>());
    typename pcl::PointCloud<T>::Ptr tmp(new pcl::PointCloud<T>());

    pcl::PointCloud<T> out; // 返回值（按值返回）

    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float max_z = -std::numeric_limits<float>::infinity();
    float min_x =  std::numeric_limits<float>::infinity();
    float min_y =  std::numeric_limits<float>::infinity();
    float min_z =  std::numeric_limits<float>::infinity();

    std::map<int, std::map<int, typename pcl::PointCloud<T>::Ptr>> meta_group;

    for (const auto& pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        continue;

      pc_rigid->push_back(pt);

      max_x = std::max(max_x, pt.x); max_y = std::max(max_y, pt.y); max_z = std::max(max_z, pt.z);
      min_x = std::min(min_x, pt.x); min_y = std::min(min_y, pt.y); min_z = std::min(min_z, pt.z);

      int col = static_cast<int>(pt.x / resolution + signf(pt.x) * 0.5f);
      int row = static_cast<int>(pt.y / resolution + signf(pt.y) * 0.5f);

      auto& cell = meta_group[row][col];
      if (!cell) cell.reset(new pcl::PointCloud<T>());
      cell->push_back(pt);
    }

    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);

    for (auto& row_group : meta_group) {
      for (auto& kv : row_group.second) {
        auto& cell_cloud = kv.second;
        if (!cell_cloud || cell_cloud->empty()) continue;

        voxel.setInputCloud(cell_cloud);
        tmp->clear();
        voxel.filter(*tmp);     // 写到 tmp
        out += *tmp;            // 叠加
      }
    }

    return out;
    }
  } // namespace ds
}   // namespace teaser_agatha


int main(int argc, char** argv)
{
  // --- 1) 定义匿名函数：把 PCLPointCloud2 转成 PointXYZINormal（缺省字段自动填0） ---
  auto ply2pcd = [](const pcl::PCLPointCloud2& in,
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr& out) -> bool {
    try {
      // 先保证有坐标：无论 PLY 是否有强度/法向，坐标一定要有
      pcl::PointCloud<pcl::PointXYZ> xyz;
      pcl::fromPCLPointCloud2(in, xyz);  // 若缺少 x/y/z，这里会抛异常

      out.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
      out->points.reserve(xyz.size());

      // 统一拷贝坐标，强度/法向若在 PLY 中不存在，则置为 0
      for (const auto& p : xyz) {
        pcl::PointXYZINormal q;
        q.x = p.x; q.y = p.y; q.z = p.z;
        q.intensity = 0.0f;
        q.normal_x = q.normal_y = q.normal_z = 0.0f;
        out->points.emplace_back(q);
      }
      out->width  = static_cast<std::uint32_t>(out->points.size());
      out->height = 1;
      out->is_dense = xyz.is_dense;
      return true;
    } catch (const std::exception& e) {
      std::cerr << "[ply2pcd] Convert failed: " << e.what() << "\n";
      return false;
    }
  };

  ros::init(argc, argv, "teaser_muilt_reg");
  ros::NodeHandle nh("~");

  // XXX  ---------- 雷达静态长曝光点云合成 ----------

  bool Livoxlidar_deal = true;
  if (Livoxlidar_deal == true)
  {
    LidarFrameSyn syn("/livox/lidar");

    // 例如：从 rosbag 中叠加最多 5 秒点云
    auto cloud = syn.buildLongExposureCloudFromBag("/home/kilox/cloud_mapping/src/teaser_muilt_reg/data/lidar_output/base1.bag", 5.0);

    // 导出 PCD（可选）
    syn.saveCloudToPCD("/home/kilox/cloud_mapping/src/teaser_muilt_reg/data/livox_lidar/base1.pcd");
  }
  

  // ---------- 1) 读取参数 ----------
  // FIXME 测试D21
  // std::string src_pcd = "/home/kilox/cloud_mapping/src/teaser_muilt_reg/data/clean_data/render_color_point_scene_2.pcd";
  // std::string tgt_pcd = "/home/kilox/cloud_mapping/src/teaser_muilt_reg/data/clean_data/render_color_point_car_1.pcd";
  // FIXME 测试用长曝光点云
  std::string src_pcd = "/home/kilox/cloud_mapping/src/teaser_muilt_reg/data/lidar_output/base.pcd";
  std::string tgt_pcd = "/home/kilox/cloud_mapping/src/teaser_muilt_reg/data/lidar_output/base1.pcd";

  // std::string tgt_pcd = "/home/kilox/catkin_r3live/data/fire2/shitang_map.pcd";
  // std::string src_pcd/*out_pcd1 转换ply时配合使用*/ = "/home/kilox/cloud_mapping/src/teaser_muilt_reg/data/output/transformed_cloud_ply_1.pcd";
  // std::string tgt_pcd/*out_pcd2*/ = "/home/kilox/cloud_mapping/src/teaser_muilt_reg/data/output/transformed_cloud_ply_2.pcd";
  std::string out_pcd1 = "/home/kilox/cloud_mapping/src/teaser_muilt_reg/data/output/transformed_cloud_ply_1.pcd";
  std::string out_pcd2= "/home/kilox/cloud_mapping/src/teaser_muilt_reg/data/output/transformed_cloud_ply_2.pcd";

  std::string out_transformed_full = "/home/kilox/cloud_mapping/src/teaser_muilt_reg/data/output/transformed_cloud_test3.pcd";
  std::string out_trans_final      = "/home/kilox/cloud_mapping/src/teaser_muilt_reg/data/output/trans_form_final.pcd";

  double leaf_xyz_inormal = 0.02;   // 第一阶段：对 target(PointXYZINormal) 做体素滤波
  double leaf_xyz         = 0.10;   // 第二阶段：对 XYZ 再体素
  double teaser_scale     = 12.0;   //XXX 11.17 10.0你代码里 ld_map_icp_teaser_scale 

  nh.param<std::string>("src_pcd", src_pcd, src_pcd/* 默认值*/);
  nh.param<std::string>("tgt_pcd", tgt_pcd, tgt_pcd);
  nh.param<std::string>("out_transformed_full", out_transformed_full, out_transformed_full);
  nh.param<std::string>("out_trans_final", out_trans_final, out_trans_final);
  nh.param<double>("leaf_xyz_inormal", leaf_xyz_inormal, leaf_xyz_inormal);
  nh.param<double>("leaf_xyz", leaf_xyz, leaf_xyz);
  nh.param<double>("teaser_scale", teaser_scale, teaser_scale);

  ROS_INFO_STREAM("src_pcd: " << src_pcd);
  ROS_INFO_STREAM("tgt_pcd: " << tgt_pcd);
  ROS_INFO_STREAM("leaf_xyz_inormal: " << leaf_xyz_inormal << " | leaf_xyz: " << leaf_xyz << " | teaser_scale: " << teaser_scale);
  
  // ---------- 2) 读取 PCD ----------
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZINormal>);

  if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(src_pcd, *cloud_source) == -1) {
    ROS_ERROR_STREAM("=======> source pcd reading failed: " << src_pcd);
    return 1;
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(tgt_pcd, *cloud_target) == -1) {
    ROS_ERROR_STREAM("=======> target pcd reading failed: " << tgt_pcd);
    return 1;
  }
  // ---------- 2.5) 格式转换 干净PCD ----------
  {
  // pcl::PCLPointCloud2 cloud2;
  // pcl::io::loadPCDFile(tgt_pcd, cloud2);

  // pcl::PointCloud<pcl::PointXYZINormal> clean;
  // clean.points.reserve(cloud2.width * cloud2.height);

  // pcl::PointCloud<pcl::PointXYZ> tmp_xyz;
  // pcl::fromPCLPointCloud2(cloud2, tmp_xyz);

  // for (auto &p : tmp_xyz.points) {
  //   pcl::PointXYZINormal q;
  //   q.x = p.x;
  //   q.y = p.y;
  //   q.z = p.z;
  //   q.intensity = 0.f;
  //   q.normal_x = q.normal_y = q.normal_z = 0.f;
  //   q.curvature = 0.f;
  //   clean.points.push_back(q);
  // }
  // clean.width  = clean.points.size();
  // clean.height = 1;
  // pcl::io::savePCDFileBinary("/home/kilox/cloud_mapping/src/teaser_muilt_reg/data/output/clean_car_22.pcd", clean);
  }
  // ---------- 2.5) end 格式转换 PCD ----------
  
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::io::loadPCDFile(src_pcd, *cloud_src_xyz);
  // pcl::io::loadPCDFile(tgt_pcd, *cloud_tgt_xyz);

  // pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZINormal>);
  // cloud_source->points.reserve(cloud_src_xyz->size());
  // for (const auto& p : cloud_src_xyz->points) {
  //   pcl::PointXYZINormal q;
  //   q.x = p.x; q.y = p.y; q.z = p.z;
  //   q.intensity = 0.0f;
  //   q.normal_x = q.normal_y = q.normal_z = 0.0f;
  //   q.curvature = 0.0f;
  //   cloud_source->points.push_back(q);
  // }
  // cloud_source->width = cloud_source->points.size();
  // cloud_source->height = 1;
  // cloud_source->is_dense = false;

  // pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZINormal>);
  // cloud_target->points.reserve(cloud_tgt_xyz->size());
  // for (const auto& p : cloud_tgt_xyz->points) {
  //   pcl::PointXYZINormal q;
  //   q.x = p.x; q.y = p.y; q.z = p.z;
  //   q.intensity = 0.0f;
  //   q.normal_x = q.normal_y = q.normal_z = 0.0f;
  //   q.curvature = 0.0f;
  //   cloud_target->points.push_back(q);
  // }
  // cloud_target->width = cloud_target->points.size();
  // cloud_target->height = 1;
  // cloud_target->is_dense = false;

  ROS_INFO_STREAM("Loaded source: " << cloud_source->size() << " | target: " << cloud_target->size());

  // ---------- 3) 第一阶段：对 target(INormal) 体素降采样 ----------
  
  // pcl::VoxelGrid<pcl::PointXYZINormal> vg_in;
  // vg_in.setLeafSize(leaf_xyz_inormal, leaf_xyz_inormal, leaf_xyz_inormal);
  // pcl::PointCloud<pcl::PointXYZINormal>::Ptr sample_target(new pcl::PointCloud<pcl::PointXYZINormal>);
  // vg_in.setInputCloud(cloud_target);
  // vg_in.filter(*sample_target);
  // ROS_INFO_STREAM("sample_target(INormal) ds size: " << sample_target->size());

  // ---------- 3.5) PLY转PCD ----------
  if (false)
  {
    // std::string src_ply = "/home/kilox/cloud_mapping/src/TEASER-plusplus_or/examples/example_data/3dmatch_sample/cloud_bin_2.ply";
    // std::string tgt_ply = "/home/kilox/cloud_mapping/src/TEASER-plusplus_or/examples/example_data/3dmatch_sample/cloud_bin_36.ply";
    // pcl::PCLPointCloud2 src_, tgt_;
    // if (pcl::io::loadPLYFile(src_ply, src_) < 0) {std::cerr << "[main] Cannot open PLY: " << src_ply << "\n";return 2;}
    // if (pcl::io::loadPLYFile(tgt_ply, tgt_) < 0) {std::cerr << "[main] Cannot open PLY: " << tgt_ply << "\n";return 2;}
    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZINormal>);
    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZINormal>);
    // if (!ply2pcd(src_, cloud_source)) {std::cerr << "[main] Convert failed.\n";return 3;}
    // if (!ply2pcd(tgt_, cloud_target)) {std::cerr << "[main] Convert failed.\n";return 3;}
    // pcl::io::savePCDFileBinary(out_ply1, cloud_source);
    // pcl::io::savePCDFileBinary(out_ply2, cloud_target);
    // --- 2) I/O 路径 ---
      const std::string src_ply = "/home/kilox/cloud_mapping/src/TEASER-plusplus_or/examples/example_data/3dmatch_sample/cloud_bin_2.ply";
      const std::string tgt_ply = "/home/kilox/cloud_mapping/src/TEASER-plusplus_or/examples/example_data/3dmatch_sample/cloud_bin_36.ply";
      // const std::string out_pcd1 = "/home/kilox/cloud_mapping/src/TEASER-plusplus_or/examples/example_data/3dmatch_sample/cloud_bin_2.pcd";
      // const std::string out_pcd2 = "/home/kilox/cloud_mapping/src/TEASER-plusplus_or/examples/example_data/3dmatch_sample/cloud_bin_36.pcd";

      // --- 3) 读取 PLY 为 PCLPointCloud2 ---
      pcl::PCLPointCloud2 src_blob, tgt_blob;
      if (pcl::io::loadPLYFile(src_ply, src_blob) < 0) {
        std::cerr << "[main] Cannot open PLY: " << src_ply << "\n";
        return 2;
      }
      if (pcl::io::loadPLYFile(tgt_ply, tgt_blob) < 0) {
        std::cerr << "[main] Cannot open PLY: " << tgt_ply << "\n";
        return 2;
      }

      // --- 4) 转成 PointXYZINormal ---
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source, cloud_target;
      if (!ply2pcd(src_blob, cloud_source)) {
        std::cerr << "[main] Convert failed for " << src_ply << "\n";
        return 3;
      }
      if (!ply2pcd(tgt_blob, cloud_target)) {
        std::cerr << "[main] Convert failed for " << tgt_ply << "\n";
        return 3;
      }

      // --- 5) 保存为 PCD（二进制） ---
      if (pcl::io::savePCDFileBinary(out_pcd1, *cloud_source) != 0) {
        std::cerr << "[main] Save failed: " << out_pcd1 << "\n";
        return 4;
      }
      if (pcl::io::savePCDFileBinary(out_pcd2, *cloud_target) != 0) {
        std::cerr << "[main] Save failed: " << out_pcd2 << "\n";
        return 4;
      }

      std::cout << "[main] Done. Wrote:\n  " << out_pcd1 << "\n  " << out_pcd2 << "\n";
      return 0;
    
  }


  // ---------- 4) TEASER 配准（保持你的调用与顺序） ----------
  ROS_INFO("begin teaser registration");    // XXX 纯读入无操作
  Eigen::Matrix4d ext_param = map_icp_teaser_icp(cloud_source, cloud_target, static_cast<float>(teaser_scale));
  // TODO 搞清楚这个是图到物的变换还是物到图的变换
  // ---------- 5) 提取 R,t 并构造仿射变换 ----------
  Eigen::Vector3d translation = ext_param.block<3,1>(0,3).cast<double>();
  Eigen::Matrix3d rotation_matrix = ext_param.block<3,3>(0,0).cast<double>();
  Eigen::Quaterniond quaternion(rotation_matrix);
  quaternion.normalize();

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.prerotate(quaternion.cast<float>());
  transform.pretranslate(translation.cast<float>());

  // ---------- 6) 应用第一次变换到 source 并保存 ----------
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::transformPointCloud(*cloud_source, *transformed_cloud, transform);
  ROS_INFO_STREAM("transformed_cloud size: " << transformed_cloud->size());
  /** 降采样输出 */
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_merge_cloud_downsample(new pcl::PointCloud<pcl::PointXYZINormal>());
  *source_merge_cloud_downsample = teaser_agatha::ds::downsample<pcl::PointXYZINormal>(transformed_cloud, 0.05f);
  if (pcl::io::savePCDFileBinary(out_transformed_full, *source_merge_cloud_downsample) != 0) {
    ROS_WARN_STREAM("Failed to save PCD: " << out_transformed_full);
  } else {
    ROS_INFO_STREAM("Saved: " << out_transformed_full);
  }

  // // ---------- 7) 准备 XYZ 点云（与你的流程一致：从 transformed & target(INormal) 投射为 XYZ） ----------

  // pcl::PointCloud<pcl::PointXYZ>::Ptr sample_cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr sample_cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr sample_cloud_source_transform(new pcl::PointCloud<pcl::PointXYZ>);

  // sample_cloud_source->points.reserve(transformed_cloud.size());
  // for (const auto& p : transformed_cloud.points) {
  //   pcl::PointXYZ q; q.x = p.x; q.y = p.y; q.z = p.z;
  //   sample_cloud_source->points.push_back(q);
  // }
  // sample_cloud_target->points.reserve(sample_target->size());
  // for (const auto& p : sample_target->points) {
  //   pcl::PointXYZ q; q.x = p.x; q.y = p.y; q.z = p.z;
  //   sample_cloud_target->points.push_back(q);
  // }

  // // ---------- 8) 第二阶段：XYZ 体素降采样 ----------
  // pcl::VoxelGrid<pcl::PointXYZ> vg_xyz;
  // vg_xyz.setLeafSize(leaf_xyz, leaf_xyz, leaf_xyz);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr sample_target_ds(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr sample_source_ds(new pcl::PointCloud<pcl::PointXYZ>);
  // vg_xyz.setInputCloud(sample_cloud_target);
  // vg_xyz.filter(*sample_target_ds);

  // vg_xyz.setInputCloud(sample_cloud_source);
  // vg_xyz.filter(*sample_source_ds);

  // ROS_INFO_STREAM("sample_source_ds: " << sample_source_ds->size()
  //                 << " | sample_target_ds: " << sample_target_ds->size());

  // // ---------- 9) 第二阶段匹配：test_match2（与你代码一致，使用 transform 作为初始） ----------
  // Eigen::Matrix4f trans_form = test_match2(sample_source_ds, sample_target_ds, transform);

  // // 应用最终变换到原 sample_cloud_source（未 ds 的那个）
  // pcl::transformPointCloud(*sample_cloud_source, *sample_cloud_source_transform, trans_form);

  // if (pcl::io::savePCDFileBinary(out_trans_final, *sample_cloud_source_transform) != 0) {
  //   ROS_WARN_STREAM("Failed to save PCD: " << out_trans_final);
  // } else {
  //   ROS_INFO_STREAM("Saved: " << out_trans_final);
  // }

  // ROS_INFO("Done.");
  return 0;
}
