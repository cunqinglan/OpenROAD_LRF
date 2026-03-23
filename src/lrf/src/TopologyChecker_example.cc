// TopologyChecker 测试示例
// 展示如何在 LRF 中使用拓扑检查器

#include "lrf/LrfMgr.hh"
#include "lrf/TaskArranger.hh"
#include "sta/Sta.hh"

namespace lrf {

// 示例 1: 基本使用
void example_basic_usage(sta::dbSta* sta, 
                        LocalSta* local_sta,
                        rsz::Resizer* resizer,
                        ParallelLrVisitor* visitor)
{
  printf("=== Example 1: Basic Usage ===\n");
  
  // 创建 TaskArranger
  TaskArranger* arranger = new TaskArranger(sta);
  
  // 构建依赖图
  arranger->makeLevelGraph(sta->network(), resizer);
  
  // 启用拓扑检查
  arranger->enableTopologyCheck(true);
  
  // 并行访问 - 违规会自动打印
  arranger->visitOrdered(sta, local_sta, resizer, visitor);
  
  // 访问结束后会自动打印违规汇总
  
  delete arranger;
}

// 示例 2: 条件性启用（仅调试时）
void example_conditional_enable(sta::dbSta* sta,
                               LocalSta* local_sta, 
                               rsz::Resizer* resizer,
                               ParallelLrVisitor* visitor,
                               bool debug_mode)
{
  printf("=== Example 2: Conditional Enable ===\n");
  
  TaskArranger* arranger = new TaskArranger(sta);
  arranger->makeLevelGraph(sta->network(), resizer);
  
  // 仅在调试模式下启用检查
  if (debug_mode) {
    printf("Debug mode: Topology checking enabled\n");
    arranger->enableTopologyCheck(true);
  } else {
    printf("Production mode: Topology checking disabled\n");
  }
  
  arranger->visitOrdered(sta, local_sta, resizer, visitor);
  
  delete arranger;
}

// 示例 3: 手动查看违规（虽然通常不需要，因为会自动打印）
void example_manual_check(sta::dbSta* sta,
                         LocalSta* local_sta,
                         rsz::Resizer* resizer, 
                         ParallelLrVisitor* visitor)
{
  printf("=== Example 3: Manual Violation Check ===\n");
  
  TaskArranger* arranger = new TaskArranger(sta);
  arranger->makeLevelGraph(sta->network(), resizer);
  arranger->enableTopologyCheck(true);
  
  arranger->visitOrdered(sta, local_sta, resizer, visitor);
  
  // 手动查看违规（可选，因为 visitOrdered 结束时已经打印过）
  // printf("\n--- Manual violation check ---\n");
  arranger->printTopologyViolations();
  
  delete arranger;
}

// 示例 4: 环境变量控制
void example_env_control(sta::dbSta* sta,
                        LocalSta* local_sta,
                        rsz::Resizer* resizer,
                        ParallelLrVisitor* visitor)
{
  printf("=== Example 4: Environment Variable Control ===\n");
  
  TaskArranger* arranger = new TaskArranger(sta);
  arranger->makeLevelGraph(sta->network(), resizer);
  
  // 通过环境变量控制
  const char* check_env = std::getenv("LRF_CHECK_TOPOLOGY");
  bool enable_check = (check_env && std::string(check_env) == "1");
  
  if (enable_check) {
    printf("LRF_CHECK_TOPOLOGY=1: Enabling topology check\n");
    arranger->enableTopologyCheck(true);
  }
  
  arranger->visitOrdered(sta, local_sta, resizer, visitor);
  
  delete arranger;
}

} // namespace lrf

// 使用方法:
// 
// 在 shell 中:
// export LRF_CHECK_TOPOLOGY=1
// ./openroad your_script.tcl
//
// 在 TCL 脚本中:
// enable_topology_check true
// parallel_gate_sizing ...
//
// 在 C++ 代码中:
// arranger->enableTopologyCheck(true);
// arranger->visitOrdered(...);
