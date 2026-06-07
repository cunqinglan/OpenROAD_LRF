#pragma once

#include "lrf/LrfClass.hh"
#include "db_sta/dbSta.hh"
#include "sta/GraphClass.hh"

namespace rsz {
  class Resizer;
}

namespace lrf {
class TaskArranger;

// This data saves swappable cells information for LR sizing
// Information includes:
// 1. Equivalent cells for each liberty cell type
// 2. Pre-saved leakage power for each liberty cell type
// 3. pre-process type window for each liberty cell type
class ParallelLibData: sta::dbStaState
{
public:
  ParallelLibData(sta::dbSta *db_sta);
  ~ParallelLibData();

  void init(rsz::Resizer* resizer, TaskArranger *task_arranger);

  void ensureActivities();
  void makeSwappableCellsCache(rsz::Resizer* resizer);
  void preSaveLibCellLeakage(rsz::Resizer* resizer, sta::Scene* corner);
  std::vector<sta::LibertyCellSeq> *getSwappableCells(sta::LibertyCell* source_cell);

  bool SwappableCellPresaved() const { return swap_cell_presaved_; }
  bool SwappableCellLeakagePresaved() const { return swap_cell_leakage_presaved_; }
  float getCellLeakage(sta::Instance* inst, size_t index) const;

protected:
  bool swap_cell_presaved_ = false;
  bool swap_cell_leakage_presaved_ = false;
  // Map from LibertyCell* to its equivalent LibertyCellSeq*
  std::unordered_map<sta::LibertyCell*, std::vector<sta::LibertyCellSeq>> swappable_cells_cache_;
  // Map from LibertyCell* to its leakage power for each equivalent cell
  std::vector<ParallelLocalCellInfo> cell_info_vec_;
  const std::unordered_map<const sta::Instance*, VertexId> *inst_to_vid_map_;

};
}