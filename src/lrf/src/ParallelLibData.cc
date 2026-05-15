

#include <vector>
#include <chrono>

#include "ParallelLibData.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "sta/LibertyClass.hh"
#include "TaskArranger.hh"
#include "sta/Scene.hh"
#include "sta/Network.hh"
#include "power/Power.hh"
#include "sta/EquivCells.hh"

namespace lrf {

ParallelLibData::ParallelLibData(sta::dbSta *db_sta)
{
  dbStaState::init(db_sta);
}

ParallelLibData::~ParallelLibData()
{
  sta_->unregisterStaState(this);
}

void 
ParallelLibData::init(rsz::Resizer* resizer, TaskArranger *task_arranger)
{
  swappable_cells_cache_.clear();
  cell_info_vec_.clear();
  inst_to_vid_map_ = task_arranger->instToVidMap();
  auto start = std::chrono::high_resolution_clock::now();
  makeSwappableCellsCache(resizer);
  auto mid = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = mid - start;
  printf("ParallelLibData::makeSwappableCellsCache took %.3f seconds\n", diff.count());
  preSaveLibCellLeakage(resizer, nullptr);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  printf("ParallelLibData::init took %.3f seconds\n", elapsed.count());
}

void
ParallelLibData::ensureActivities()
{
  sta::Scene *corner = sta_->findScene("default");
  sta::LeafInstanceIterator* inst_iter = network_->leafInstanceIterator();
  sta::Instance* inst = nullptr;
  while (inst_iter->hasNext()) {
    inst = inst_iter->next();
    if (!network_->libertyCell(inst))
      continue;
    else
      break;
  }
  delete inst_iter;
  if (inst)
    sta_->power(inst, corner);
  else 
    throw std::runtime_error("IncreSta::ensureActivities no valid instance found to trigger activity calculation\n");
}

std::vector<sta::LibertyCellSeq> *
ParallelLibData::getSwappableCells(sta::LibertyCell* source_cell)
{
  auto it = swappable_cells_cache_.find(source_cell);
  if (it != swappable_cells_cache_.end()) {
    return &(it->second);
  } else {
    return nullptr;
  }
}

void
ParallelLibData::makeSwappableCellsCache(rsz::Resizer* resizer)
{
  // This should be used as an ensurance
  resizer->makeEquivCells();
  // First clear the existing cache
  swappable_cells_cache_.clear();
  // prepare the neccesary activities
  ensureActivities();
  const sta::LibertyCellSeq &unique_equiv_cells = sta_->equivCellsRecorder()->uniqueEquivCells();
  for (const sta::LibertyCell* source_cell : unique_equiv_cells) {
    sta::LibertyCellSeq *equive_cells = sta_->equivCells(const_cast<sta::LibertyCell*>(source_cell));
    for (sta::LibertyCell* equiv_cell : *equive_cells) {
      sta::LibertyCellSeq swappable = resizer->getSwappableCells(equiv_cell);
      swappable_cells_cache_[equiv_cell] = {swappable};
    }
  }
  swap_cell_presaved_ = true;
}

void
ParallelLibData::preSaveLibCellLeakage(rsz::Resizer* resizer, sta::Scene* corner)
{
  if (!swap_cell_presaved_) {
    printf("ParallelLibData::preSaveLibCellLeakage Error: Swappable cells not prepared yet!\n");
    makeSwappableCellsCache(resizer);
  }
  if (inst_to_vid_map_ == nullptr) {
    throw std::runtime_error("ParallelLibData::preSaveLibCellLeakage inst_to_vid_map_ is null (did you call init?)\n");
  }
  if (corner == nullptr) {
    corner = sta_->findScene("default");
  }
  cell_info_vec_.resize(inst_to_vid_map_->size());
  sta::Power* power_calc = sta_->power();
  for (const auto& pair : *inst_to_vid_map_) {
    sta::Instance* inst = const_cast<sta::Instance*>(pair.first);
    VertexId index = pair.second;
    ParallelLocalCellInfo &cell_info = cell_info_vec_[index];
    cell_info.equiv_cells = nullptr;
    cell_info.cell_leakages.clear();
    sta::LibertyCell *cell = network_->libertyCell(inst);
    if (cell == nullptr) {
      continue;
    }

    // Avoid operator[] here because it would insert a default entry.
    auto it = swappable_cells_cache_.find(cell);
    if (it == swappable_cells_cache_.end()) {
      continue;
    }
    cell_info.equiv_cells = &it->second;

    for (const sta::LibertyCellSeq& equiv_cells : *cell_info.equiv_cells) {
      cell_info.cell_leakages.emplace_back(equiv_cells.size(), 0.0f);
      for (size_t j = 0; j < equiv_cells.size(); ++j) {
        sta::LibertyCell* equiv_cell = equiv_cells[j];
        cell_info.cell_leakages.back()[j]
            = power_calc->leakagePower(inst, equiv_cell, corner);
      }
    }
  }
  swap_cell_leakage_presaved_ = true;
}


} // namespace lrf