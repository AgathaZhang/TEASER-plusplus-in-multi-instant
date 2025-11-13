/**
 * Copyright 2020, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Jingnan Shi, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <flann/flann.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "teaser/matcher.h"
#include "teaser/geometry.h"

namespace teaser {

std::vector<std::pair<int, int>> Matcher::calculateCorrespondences(
    const teaser::PointCloud& source_points, const teaser::PointCloud& target_points,
    const teaser::FPFHCloud& source_features, const teaser::FPFHCloud& target_features,
    bool use_absolute_scale, bool use_crosscheck, bool use_tuple_test, float tuple_scale) {

  Feature cloud_features;
  pointcloud_.push_back(source_points);
  pointcloud_.push_back(target_points);   // 两个点云分开放 外层vector [1]表示src [2]表示tgt

  // It compute the global_scale_ required to set correctly the search radius
  normalizePoints(use_absolute_scale);    //  尺度统一 这个缩放是否影响原图 11.05  11.06 已解决 会整体缩放
                                          /** 这里在点云零均值化：用几何中心作平移至(0,0,0)*/
  for (auto& f : source_features) {
    Eigen::VectorXf fpfh(33);
    for (int i = 0; i < 33; i++)
      fpfh(i) = f.histogram[i];
    cloud_features.push_back(fpfh);
  }
  features_.push_back(cloud_features);    // 对features_进行 装填

  cloud_features.clear();
  for (auto& f : target_features) {
    Eigen::VectorXf fpfh(33);
    for (int i = 0; i < 33; i++)
      fpfh(i) = f.histogram[i];
    cloud_features.push_back(fpfh);
  }
  features_.push_back(cloud_features);   // 两个特征* 每个点的33维直方图 [1]表示src [2]表示tgt

  advancedMatching(use_crosscheck, use_tuple_test, tuple_scale); /* 开“交叉校验”匹配, 开几何元组一致性, 元组尺度系数*/
  // 只保留同时满足 A→B 和 B→A 互相指回来的对应对，剔除单向匹配的假对
  // use_tuple_test 用小元组（如三点/四点）检查距离比、角度等是否自洽，过滤掉几何不一致的对应
  // 决定元组的空间大小/容差（相对点云尺寸或特征间距的比例），数值越大元组更“粗”、容差更宽，越小更“紧”、筛选更严格
  return corres_;
}

void Matcher::normalizePoints(bool use_absolute_scale) {
  int num = 2;
  float scale = 0;

  means_.clear();

  for (int i = 0; i < num; ++i) {
    float max_scale = 0;

    // compute mean
    Eigen::Vector3f mean;
    mean.setZero();

    int npti = pointcloud_[i].size();
    for (int ii = 0; ii < npti; ++ii) {
      Eigen::Vector3f p(pointcloud_[i][ii].x, pointcloud_[i][ii].y, pointcloud_[i][ii].z);
      mean = mean + p;
    }
    mean = mean / npti;
    means_.push_back(mean);                // 求出几何中心

    for (int ii = 0; ii < npti; ++ii) {    // 点云零均值化
      pointcloud_[i][ii].x -= mean(0);
      pointcloud_[i][ii].y -= mean(1);
      pointcloud_[i][ii].z -= mean(2);
    }

    // compute scale
    for (int ii = 0; ii < npti; ++ii) {
      Eigen::Vector3f p(pointcloud_[i][ii].x, pointcloud_[i][ii].y, pointcloud_[i][ii].z);
      float temp = p.norm(); // because we extract mean in the previous stage.
      if (temp > max_scale) {               // 这里求了一个点云向量的模长的最大值
        max_scale = temp;
      }
    }

    if (max_scale > scale) {
      scale = max_scale;
    }
  }

  // mean of the scale variation
  if (use_absolute_scale) {
    global_scale_ = 1.0f;
  } else {
    global_scale_ = scale; // second choice: we keep the maximum scale.
  }

  if (global_scale_ != 1.0f) {
    for (int i = 0; i < num; ++i) {
      int npti = pointcloud_[i].size();
      for (int ii = 0; ii < npti; ++ii) {
        pointcloud_[i][ii].x /= global_scale_;
        pointcloud_[i][ii].y /= global_scale_;
        pointcloud_[i][ii].z /= global_scale_;
      }
    }
  }
}

void Matcher::advancedMatching(bool use_crosscheck, bool use_tuple_test, float tuple_scale) {
                                          /*true 交叉检验 false 元组测试 0.95元组阈值*/
  int fi = 0; // source idx
  int fj = 1; // destination idx

  bool swapped = false;

  if (pointcloud_[fj].size() > pointcloud_[fi].size()) {      /* 场景大于目标交换点云 */
    int temp = fi;
    fi = fj;
    fj = temp;
    swapped = true;     // 已交换
  }

  int nPti = pointcloud_[fi].size();            // fi 总是大点云场景
  int nPtj = pointcloud_[fj].size();

  ///////////////////////////
  /// Build FLANNTREE
  ///////////////////////////
  KDTree feature_tree_i(flann::KDTreeSingleIndexParams(15));
  buildKDTree(features_[fi], &feature_tree_i);

  KDTree feature_tree_j(flann::KDTreeSingleIndexParams(15));    // 参数 15 通常表示叶子最大容量（leaf size）。
  buildKDTree(features_[fj], &feature_tree_j);

  std::vector<int> corres_K;                      // KD-Tree 查询返回的邻居索引列表
  std::vector<float> dis;                         // KD-Tree 查询返回的邻居距离列表   

  std::vector<std::pair<int, int>> corres;        // 最终匹配对列表
  std::vector<std::pair<int, int>> corres_cross;  // 交叉检验后的匹配对列表
  std::vector<std::pair<int, int>> corres_ij;     // i->j方向的初始匹配对列表 
  std::vector<std::pair<int, int>> corres_ji;     // j->i方向的初始匹配对列表

  ///////////////////////////
  /// INITIAL MATCHING
  ///////////////////////////
  std::vector<int> i_to_j(nPti, -1);              // 记录 i 点云中每个点对应的 j 点云点的索引，初始为 -1（无对应）
  for (int j = 0; j < nPtj; j++) {                // 遍历 j 小团 点云的每个点
    searchKDTree(&feature_tree_i, features_[fj][j], corres_K, dis, 10);   // 在 i 大团 点云的特征树中搜索与 j 点最相似的特征，找10个最近邻
    int i = corres_K[0];      // 比如在i大团中找到12号 点最接近 j 点
    if (i_to_j[i] == -1) {    // 初始化为 i 大团 还没放入
      searchKDTree(&feature_tree_j, features_[fi][i], corres_K, dis, 10);
      int ij = corres_K[0];   // 在j小团中找到了最接近i大团中12号的 1号
      i_to_j[i] = ij;         // i中12号点对应 = j中1号点
    }
    corres_ji.push_back(std::pair<int, int>(i, j));  // <12,1> corres_ji 数量是 小团j的全部数量

  }
  std::cout << "corres_ji size 小团j的全部数量匹配: " << corres_ji.size() << std::endl;
  for (int i = 0; i < nPti; i++) {
    if (i_to_j[i] != -1)
      corres_ij.push_back(std::pair<int, int>(i, i_to_j[i])); // <12,1> 大团i 首次被抓到的匹配
  }                                                           // 当某对 (i,j) 恰好是互为最近邻时重复 这里可以用合并时做去重（如用 unordered_set<pair<int,int>>)
  std::cout << "corres_ij size 大团i的首次被catch数量匹配: " << corres_ij.size() << std::endl;
  int ncorres_ij = corres_ij.size();                          // 这里corres_ij数量是 <= corres_ji的
  int ncorres_ji = corres_ji.size();

  // corres = corres_ij + corres_ji;
  for (int i = 0; i < ncorres_ij; ++i)
    corres.push_back(std::pair<int, int>(corres_ij[i].first, corres_ij[i].second));
  for (int j = 0; j < ncorres_ji; ++j)
    corres.push_back(std::pair<int, int>(corres_ji[j].first, corres_ji[j].second));
    std::cout << "corres 匹配总和: " << corres.size() << std::endl;
  ///////////////////////////
  /// CROSS CHECK
  /// input : corres_ij, corres_ji
  /// output : corres
  ///////////////////////////
  if (use_crosscheck) {                             // 开交叉检验
    std::cout << "CROSS CHECK" << std::endl;
    // build data structure for cross check
    corres.clear();
    corres_cross.clear();
    std::vector<std::vector<int>> Mi(nPti);         // Mi[i] 存储 i 点云中点 i 对应的 j 点云点索引列表
    std::vector<std::vector<int>> Mj(nPtj);

    int ci, cj;
    for (int i = 0; i < ncorres_ij; ++i) {          // 首次i中->j匹配
      ci = corres_ij[i].first;
      cj = corres_ij[i].second;
      Mi[ci].push_back(cj);                         // 大团中有被匹配到的 小团中的位置放内层vector
    }
    for (int j = 0; j < ncorres_ji; ++j) {          // 小团j中->i全部点匹配
      ci = corres_ji[j].first;
      cj = corres_ji[j].second;
      Mj[cj].push_back(ci);                         // 小团中有被匹配到的 大团中的位置放内层vector
    }

    // cross check
    for (int i = 0; i < nPti; ++i) {                                // 遍历大团i的每个点
      for (int ii = 0; ii < Mi[i].size(); ++ii) {                   // 遍历 i 点云中点 i 对应的所有 j 点云点索引
        int j = Mi[i][ii];                
        for (int jj = 0; jj < Mj[j].size(); ++jj) {
          if (Mj[j][jj] == i) {
            corres.push_back(std::pair<int, int>(i, j));
            corres_cross.push_back(std::pair<int, int>(i, j));
          }
        }
      }
    }
    std::cout << "corres_cross 交叉检测剩余数量匹配: " << corres_cross.size() << std::endl;
  } else {
    std::cout << "Skipping Cross Check." << std::endl;
  }

  ///////////////////////////
  /// TUPLE CONSTRAINT
  /// input : corres
  /// output : corres
  /** 核心做法是：从匹配对里抽取小元组，检查刚体不变量
   *（如边长比、夹角、面积比/共面性、法向朝向等）
   * 在两端是否一致；不一致的对应整体剔除。这样能强力压制对称体、
   * 重复结构、局部相似导致的伪匹配*/
  ///////////////////////////
  if (use_tuple_test && tuple_scale != 0) {       /** 元组几何一致性检查 用小元组（通常三点/四点）做几何一致性筛选*/
    std::cout << "TUPLE CONSTRAINT" << std::endl;
    srand(time(NULL));                      // 给 C 标准库的随机数生成器设置种子
    int rand0, rand1, rand2;
    int idi0, idi1, idi2;
    int idj0, idj1, idj2;
    float scale = tuple_scale;
    int ncorr = corres.size();              // 匹配数
    int number_of_trial = ncorr * 100;      // 匹配总和 *100
    std::vector<std::pair<int, int>> corres_tuple;

    for (int i = 0; i < number_of_trial; i++) {
      rand0 = rand() % ncorr;
      rand1 = rand() % ncorr;
      rand2 = rand() % ncorr;

      idi0 = corres[rand0].first;
      idj0 = corres[rand0].second;
      idi1 = corres[rand1].first;
      idj1 = corres[rand1].second;
      idi2 = corres[rand2].first;
      idj2 = corres[rand2].second;

      // collect 3 points from i-th fragment
      Eigen::Vector3f pti0 = {pointcloud_[fi][idi0].x, pointcloud_[fi][idi0].y,
                              pointcloud_[fi][idi0].z};
      Eigen::Vector3f pti1 = {pointcloud_[fi][idi1].x, pointcloud_[fi][idi1].y,
                              pointcloud_[fi][idi1].z};
      Eigen::Vector3f pti2 = {pointcloud_[fi][idi2].x, pointcloud_[fi][idi2].y,
                              pointcloud_[fi][idi2].z};

      float li0 = (pti0 - pti1).norm();
      float li1 = (pti1 - pti2).norm();
      float li2 = (pti2 - pti0).norm();

      // collect 3 points from j-th fragment
      Eigen::Vector3f ptj0 = {pointcloud_[fj][idj0].x, pointcloud_[fj][idj0].y,
                              pointcloud_[fj][idj0].z};
      Eigen::Vector3f ptj1 = {pointcloud_[fj][idj1].x, pointcloud_[fj][idj1].y,
                              pointcloud_[fj][idj1].z};
      Eigen::Vector3f ptj2 = {pointcloud_[fj][idj2].x, pointcloud_[fj][idj2].y,
                              pointcloud_[fj][idj2].z};

      float lj0 = (ptj0 - ptj1).norm();
      float lj1 = (ptj1 - ptj2).norm();
      float lj2 = (ptj2 - ptj0).norm();

      if ((li0 * scale < lj0) && (lj0 < li0 / scale) && (li1 * scale < lj1) &&
          (lj1 < li1 / scale) && (li2 * scale < lj2) && (lj2 < li2 / scale)) {
        corres_tuple.push_back(std::pair<int, int>(idi0, idj0));
        corres_tuple.push_back(std::pair<int, int>(idi1, idj1));
        corres_tuple.push_back(std::pair<int, int>(idi2, idj2));
      }
    }
    corres.clear();

    for (size_t i = 0; i < corres_tuple.size(); ++i)
    corres.push_back(std::pair<int, int>(corres_tuple[i].first, corres_tuple[i].second));
  } else {
    std::cout << "Skipping Tuple Constraint." << std::endl;
  }

  if (swapped) {
    std::vector<std::pair<int, int>> temp;
    for (size_t i = 0; i < corres.size(); i++)
      temp.push_back(std::pair<int, int>(corres[i].second, corres[i].first));
    corres.clear();
    corres = temp;
  }
  corres_ = corres;

  ///////////////////////////
  /// ERASE DUPLICATES
  /// input : corres_
  /// output : corres_
  ///////////////////////////
  std::sort(corres_.begin(), corres_.end());
  corres_.erase(std::unique(corres_.begin(), corres_.end()), corres_.end());
  std::cout << "元组一致性检查后: " << corres_.size() << std::endl;
}

template <typename T> 
void Matcher::buildKDTree(const std::vector<T>& data, Matcher::KDTree* tree) {
  int rows, dim;
  rows = (int)data.size();
  dim = (int)data[0].size();
  std::cout << "Building KDTree with " << rows << " points of dimension " << dim << std::endl;
  std::vector<float> dataset(rows * dim);
  flann::Matrix<float> dataset_mat(&dataset[0], rows, dim);
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < dim; j++)
      dataset[i * dim + j] = data[i][j];
  KDTree temp_tree(dataset_mat, flann::KDTreeSingleIndexParams(15));
  temp_tree.buildIndex();
  *tree = temp_tree;        // 应把数据缓存在 tree 的成员里（或用 shared_ptr 持有），避免悬挂引用
}

template <typename T>
void Matcher::searchKDTree(Matcher::KDTree* tree,       // KDTree 名指针
                          const T& input,               // 查询的输入点
                          std::vector<int>& indices,    // 返回的邻居索引列表
                          std::vector<float>& dists,    // 返回的邻居距离列表
                          int nn) {                     // 1 最近邻数量
  int rows_t = 1;
  int dim = input.size();

  std::vector<float> query;
  query.resize(rows_t * dim);
  for (int i = 0; i < dim; i++)
    query[i] = input(i);
  flann::Matrix<float> query_mat(&query[0], rows_t, dim);

  indices.resize(rows_t * nn);
  dists.resize(rows_t * nn);
  flann::Matrix<int> indices_mat(&indices[0], rows_t, nn);
  flann::Matrix<float> dists_mat(&dists[0], rows_t, nn);
  /** 在已建好的 FLANN 索引 tree 上做 k 近邻搜索*/
  tree->knnSearch(query_mat, indices_mat, dists_mat, nn, flann::SearchParams(128));
                  // 查询点集 查询结果索引矩阵 查询结果距离矩阵 邻居数量 搜索参数
  
}

} // namespace teaser
