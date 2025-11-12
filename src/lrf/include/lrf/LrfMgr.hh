

#pragma once

#include <map>
#include <set>
#include <array>
#include <optional>
#include <string>
#include <vector>
#include <unordered_map>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "dpl/Opendp.h"
#include "est/EstimateParasitics.h"
#include "grt/GlobalRouter.h"
#include "odb/dbTypes.h"
#include "rsz/OdbCallBack.hh"
#include "sta/Path.hh"
#include "sta/UnorderedSet.hh"
#include "utl/Logger.h"

namespace odb {
class dbDatabase;
class dbChip;
class dbBlock;
}  // namespace odb

namespace sta {
class dbNetwork;
class Instance;
class NetworkReader;
class Library;
class Port;
class Net;
class dbSta;
}  // namespace sta

namespace grt {
class GlobalRouter;
}  //

namespace stt {
class SteinerTreeBuilder;
}

namespace lrf {
using utl::Logger;

using grt::GlobalRouter;
using stt::SteinerTreeBuilder;

using odb::dbBlock;
using odb::dbDatabase;

using sta::dbNetwork;
using sta::dbNetworkObserver;
using sta::dbSta;
using sta::dbStaState;
using sta::Instance;
using sta::LibertyCell;
using sta::LibertyCellSeq;
using sta::Pin;

class LrfMgr : public dbStaState, public dbNetworkObserver
{
 public:
  LrfMgr(utl::Logger* logger,
          odb::dbDatabase* db,
          sta::dbSta* sta,
          SteinerTreeBuilder* stt_builder,
          GlobalRouter* global_router,
          dpl::Opendp* opendp,
          est::EstimateParasitics* estimate_parasitics);
  ~LrfMgr() override;

  // API for Lagrangian optimization
  void LagrangianOpt(int threads = 1);
  


  // General API for geting db and sta pointers
  dbNetwork* getDbNetwork() const { return db_network_; }
  sta::dbSta* getDbSta() const { return sta_; }

protected:
  void transformInstance(Instance* inst,
                         LibertyCell* new_cell);

                         
private:
  // from sta::dbNetworkObserver callbacks
  void postReadLiberty() override;

  Logger* logger_ = nullptr;
  est::EstimateParasitics* estimate_parasitics_ = nullptr;
  SteinerTreeBuilder* stt_builder_ = nullptr;
  GlobalRouter* global_router_ = nullptr;
  dbNetwork* db_network_ = nullptr;
  dbDatabase* db_ = nullptr;
  dbBlock* block_ = nullptr;
  int dbu_ = 0;
  const Pin* debug_pin_ = nullptr;

  odb::Rect core_;
  bool core_exists_ = false;
  sta::dbSta* sta_ = nullptr;
  std::unordered_map<LibertyCell*, LibertyCellSeq> swappable_cells_cache_;
};


}  // namespace lrf
