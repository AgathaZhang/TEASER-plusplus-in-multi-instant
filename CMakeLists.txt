cmake_minimum_required(VERSION 3.10)
project(teaser_muilt_reg LANGUAGES CXX C)

# ========= 基本编译选项 =========
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(-g) # 设置编译类型为 Debug，并添加调试信息
set(CMAKE_BUILD_TYPE Release)

# 新版 CMake 对旧 Python 模块的提醒（抑制兼容性噪音）
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# ========= ROS & 依赖 =========
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 QUIET)
find_package(OpenMP)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros
)

# ========= TEASER++ 源码（Vendored：随包一起编译）以 CMakeLists 所在目录为基准 =========

set(TEASER_ROOT         ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/TEASER-plusplus/teaser)
set(TEASER_INC_DIR      ${TEASER_ROOT}/include)                  # 1 核心头文件
set(TEASER_SRC_DIR      ${TEASER_ROOT}/src)                      # 1 核心源文件实现
set(TEASER_BENCH_DIR    ${TEASER_ROOT}/spectra/benchmark)        # **benchmark 根目录

set(COMPOUND_DIR        ${CMAKE_CURRENT_SOURCE_DIR}/include)      # **compound agatha自定义工程组件根目录

set(TEASER_TINYPLY_TP_INC   ${TEASER_ROOT}/tinyply/third-party)  # 2 tinyply 第三方头文件

set(TEASER_PMC_DIR      ${TEASER_ROOT}/pmc)                      # **pmc 根目录
set(TEASER_SPECTRA_DIR  ${TEASER_ROOT}/spectra)                  # **spectra 根目录
set(TEASER_PMC_INC      ${TEASER_PMC_DIR}/include)               # 3 pmc 模块头文件
set(TEASER_SPECTRA_INC  ${TEASER_SPECTRA_DIR}/include)           # 4 spectra 模块头文件

set(TEASER_EXAMPLES     ${TEASER_ROOT}/../examples/teaser_cpp_fpfh)   # /*teaser_cpp_fpfh.cc*/

# IF(EXISTS "${TEASER_EXAMPLES}")
#   file(GLOB TEASER_EXAMPLES_INC  "${TEASER_EXAMPLES}/*.hpp")
#   list(APPEND TEASER_EXAMPLES_INC_ALL ${TEASER_EXAMPLES_INC})
# ENDIF()

# 头文件整合
set(_TEASER_INC_CANDIDATES
  ${COMPOUND_DIR}              # **compound 自定义工程组件根目录
  ${TEASER_INC_DIR}            # 1 核心头文件
  ${TEASER_TINYPLY_TP_INC}     # 2 tinyply 第三方头文件
  ${TEASER_PMC_INC}            # 3 pmc 模块头文件
  ${TEASER_SPECTRA_INC}        # 4 spectra 模块头文件
  # ${TEASER_EXAMPLES_INC_ALL}   # 5 examples 模块头文件 以前读取了.hpp错误，现在改为整体目录包含
  ${TEASER_EXAMPLES}           # 5 examples 模块头文件
)

set(TEASER_INC_DIRS_EXISTING "")          # 最终头文件目录列表 
foreach(_inc ${_TEASER_INC_CANDIDATES})
  if(EXISTS "${_inc}")
    list(APPEND TEASER_INC_DIRS_EXISTING "${_inc}")
  endif()
endforeach()

# 源文件收集：teaser/spectra/benchmark
set(TEASER_SRC_ALL "")
if(EXISTS "${TEASER_BENCH_DIR}")
  AUX_SOURCE_DIRECTORY(${TEASER_BENCH_DIR} TEASER_SRC_BENCHMARK)
  list(APPEND TEASER_SRC_ALL ${TEASER_SRC_BENCHMARK})
else()
  message(FATAL_ERROR "[teaser] missing benchmark src dir: ${TEASER_BENCH_DIR}")
endif()

# 源文件收集：teaser/src
set(TEASER_SRC_ALL "")
if(EXISTS "${TEASER_SRC_DIR}")
  AUX_SOURCE_DIRECTORY(${TEASER_SRC_DIR} TEASER_SRC_CORE)
  list(APPEND TEASER_SRC_ALL ${TEASER_SRC_CORE})
else()
  message(FATAL_ERROR "[teaser] missing core src dir: ${TEASER_SRC_DIR}")
endif()

# 源文件收集：examples
if(EXISTS "${TEASER_EXAMPLES}")
  file(GLOB TEASER_EXAMPLES_CC  "${TEASER_EXAMPLES}/*.cc")
  list(APPEND TEASER_SRC_ALL ${TEASER_EXAMPLES_CC})
ENDIF()

# 源文件收集：pmc（TEASER 的最大团求解模块，某些 fork 在此）
if(EXISTS "${TEASER_PMC_DIR}")
  file(GLOB TEASER_SRC_PMC_CPP "${TEASER_PMC_DIR}/*.cpp")
  file(GLOB TEASER_SRC_PMC_CC  "${TEASER_PMC_DIR}/*.cc")
  file(GLOB TEASER_SRC_PMC_C  "${TEASER_PMC_DIR}/*.c")
  set(TEASER_SRC_PMC ${TEASER_SRC_PMC_CPP} ${TEASER_SRC_PMC_CC} ${TEASER_SRC_PMC_C})
  # 过滤掉可能包含 main() 的测试/示例文件
  # list(FILTER TEASER_SRC_PMC EXCLUDE REGEX ".*(test|example|demo|bench).*\\.c(c|pp)$")
  # list(FILTER TEASER_SRC_PMC EXCLUDE REGEX "(?i).*(test|example|demo|bench).*\\.(c|cc|cpp)$")
  list(FILTER TEASER_SRC_PMC EXCLUDE REGEX ".*[\\/](libpmc_test)\\.cpp$")
  list(APPEND TEASER_SRC_ALL ${TEASER_SRC_PMC})
  if(NOT TEASER_SRC_PMC_CPP AND NOT TEASER_SRC_PMC_CC)
    message(FATAL_ERROR "[teaser] pmc dir exists but has no .cpp/.cc sources: ${TEASER_PMC_DIR}")
  endif()
else()
  message(FATAL_ERROR "[teaser] missing pmc dir: ${TEASER_PMC_DIR}")
endif()

# （可选）spectra/benchmark
# if(EXISTS "${TEASER_SPECTRA_DIR}/benchmark")
#   AUX_SOURCE_DIRECTORY(${TEASER_SPECTRA_DIR}/benchmark TEASER_SRC_SPECTRA_BENCH)
#   list(APPEND TEASER_SRC_ALL ${TEASER_SRC_SPECTRA_BENCH})
# endif()

# ========= 你的工程源码 =========
set(SRC_NODE
  src/teaser_muilt_reg_node.cpp
  src/lidar_compound.cpp
)

# 若你的实现文件存在则自动加入（避免未定义引用）
# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/map_icp_teaser_icp.cpp")
#   list(APPEND SRC_NODE src/map_icp_teaser_icp.cpp)
# endif()
# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/test_match2.cpp")
#   list(APPEND SRC_NODE src/test_match2.cpp)
# endif()

# ========= 目标可执行文件 =========
set(ALL_SOURCES ${SRC_NODE} ${TEASER_SRC_ALL})
# list(LENGTH ALL_SOURCES _num_src)
# if(_num_src EQUAL 0)
#   message(FATAL_ERROR "No sources found for teaser_muilt_reg_node")
# endif()

add_executable(teaser_muilt_reg_node ${ALL_SOURCES})

# 头文件包含（统一放 target 作用域）
target_include_directories(teaser_muilt_reg_node PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${TEASER_INC_DIRS_EXISTING}
  ${EIGEN3_INCLUDE_DIR}
)

# PCL 宏定义（如 NDEBUG / Eigen 相关）
add_definitions(${PCL_DEFINITIONS})

# OpenMP（关键字风格，统一一次性链接）
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found: ${OpenMP_CXX_VERSION}")
  target_link_libraries(teaser_muilt_reg_node PUBLIC OpenMP::OpenMP_CXX)
else()
  message(WARNING "OpenMP not found; falling back to -fopenmp / -lgomp")
  target_compile_options(teaser_muilt_reg_node PUBLIC -fopenmp)
  target_link_libraries(teaser_muilt_reg_node PUBLIC gomp)
endif()

# 第三方库链接（关键字风格，集中一处）
target_link_libraries(teaser_muilt_reg_node PUBLIC
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(teaser_muilt_reg_node ${catkin_EXPORTED_TARGETS})

# ========= 安装（可选） =========
# install(TARGETS teaser_muilt_reg_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   FILES_MATCHING PATTERN "*.launch")
