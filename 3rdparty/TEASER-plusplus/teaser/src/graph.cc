/**
 * Copyright 2020, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Jingnan Shi, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#include "teaser/graph.h"
#include "pmc/pmc.h"

std::vector<int> teaser::MaxCliqueSolver::findMaxClique(teaser::Graph graph) {

  // Handle deprecated field
  if (!params_.solve_exactly) {
    params_.solver_mode = CLIQUE_SOLVER_MODE::PMC_HEU;
  }

  // Create a PMC graph from the TEASER graph    /** PMC = Parallel Maximum Clique（并行最大团）*/
  std::vector<int> edges;                        // undirected edges list
  std::vector<long long> vertices;               // vertex pointer list   
  vertices.push_back(edges.size());              // starting from vertex 0

  const auto all_vertices = graph.getVertices(); // get all vertex ids
  for (const auto& i : all_vertices) {
    const auto& c_edges = graph.getEdges(i);
    edges.insert(edges.end(), c_edges.begin(), c_edges.end());
    vertices.push_back(edges.size());
  }

  // Use PMC to calculate
  pmc::pmc_graph G(vertices, edges);   // CSR（压缩行存储）格式的图结构

  // Prepare PMC input
  // TODO: Incorporate this to the constructor
  pmc::input in;
  in.algorithm = 0;
  in.threads = params_.num_threads;
  in.experiment = 0;
  in.lb = 0;
  in.ub = 0;
  in.param_ub = 0;
  in.adj_limit = 20000;
  in.time_limit = params_.time_limit;
  in.remove_time = 4;
  in.graph_stats = false;
  in.verbose = false;
  in.help = false;
  in.MCE = false;
  in.decreasing_order = false;
  in.heu_strat = "kcore";
  in.vertex_search_order = "deg";

  // vector to represent max clique
  std::vector<int> C;       // 最大团的顶点集合 顶点 ID 列表
  // TODO 原理未看 11.17
  // upper-bound of max clique
  G.compute_cores();        // 对图做 k-core 分解（逐步删去度 < k 的点）为每个顶点计算其 core number。
  auto max_core = G.get_max_core(); // 图的最大 core number

  TEASER_DEBUG_INFO_MSG("Max core number: " << max_core);
  TEASER_DEBUG_INFO_MSG("Num vertices: " << vertices.size());

  // check for k-core heuristic threshold
  // check whether threshold equals 1 to short circuit the comparison
  if (params_.solver_mode == CLIQUE_SOLVER_MODE::KCORE_HEU &&
      params_.kcore_heuristic_threshold != 1 &&                        // 度数乘法系数为0.5 似乎是大于一半的点
      max_core > static_cast<int>(params_.kcore_heuristic_threshold *             
                                  static_cast<double>(all_vertices.size()))) {
    TEASER_DEBUG_INFO_MSG("Using K-core heuristic finder.");
    // remove all nodes with core number less than max core number
    // k_cores is a vector saving the core number of each vertex

    /** 核数（core number）定义：
    对无向图，k-core 是“每个顶点度数都 ≥ k”的最大子图。一个顶点的核数就是它能属于的最大 k 值
    （从图里不断剥去度 < k 的点，直到稳定；该点最后停留的最大 k）。*/
    auto k_cores = G.get_kcores();
    for (int i = 1; i < k_cores->size(); ++i) {
      // Note: k_core has size equals to num vertices + 1
      if ((*k_cores)[i] >= max_core) {
        C.push_back(i - 1);
      }
    }
    return C;
  }

  if (in.ub == 0) {
    in.ub = max_core + 1;
  }

  // lower-bound of max clique
  if (in.lb == 0 && in.heu_strat != "0") { // skip if given as input
    pmc::pmc_heu maxclique(G, in);
    in.lb = maxclique.search(G, C);
  }

  assert(in.lb != 0);
  if (in.lb == 0) {
    // This means that max clique has a size of one
    TEASER_DEBUG_ERROR_MSG("Max clique lower bound equals to zero. Abort.");
    return C;
  }

  if (in.lb == in.ub) {
    return C;
  }

  // Optional exact max clique finding
  if (params_.solver_mode == CLIQUE_SOLVER_MODE::PMC_EXACT) {
    // The following methods are used:
    // 1. k-core pruning
    // 2. neigh-core pruning/ordering
    // 3. dynamic coloring bounds/sort
    // see the original PMC paper and implementation for details:
    // R. A. Rossi, D. F. Gleich, and A. H. Gebremedhin, “Parallel Maximum Clique Algorithms with
    // Applications to Network Analysis,” SIAM J. Sci. Comput., vol. 37, no. 5, pp. C589–C616, Jan.
    // 2015.
    if (G.num_vertices() < in.adj_limit) {
      G.create_adj();
      pmc::pmcx_maxclique finder(G, in);
      finder.search_dense(G, C);
    } else {
      pmc::pmcx_maxclique finder(G, in);
      finder.search(G, C);
    }
  }

  return C;
}
