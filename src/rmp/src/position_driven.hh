#include "cut/logic_cut.h"
#include "Strategy.hh"

namespace rmp {
class PositionDrivenStrategy : public ExtractLocalWindow
{
 public:
  PositionDrivenStrategy(utl::Logger *logger) : ExtractLocalWindow(logger), candidate_cut_({}, {}, {}), worst_cut_({}, {}, {}) {}
    ~PositionDrivenStrategy() override = default;

  void setCandidateCut(const cut::LogicCut &cut) {
    candidate_cut_ = cut;
  }
  void setWorstCut(const cut::LogicCut &cut) {
    worst_cut_ = cut;
  }
  
  void remap(SeqRemapper& remapper);
  
  sta::Slack evaluateSolution(abc::Map_MappingSolution_t* pSolution,
                              abc::Map_Man_t* pMan,
                              abc::Abc_Ntk_t* pOriginalNetwork,
                              cut::LogicCut& candidate_cut,
                              SeqRemapper& remapper);
  
  //void positionDrivenRemap (SeqRemapper& remapper);
  sta::Vertex* getFarthestOutputVertex(
      SeqRemapper& remapper);
  sta::Vertex* getWorstVertex(
      SeqRemapper& remapper);
  void extractCandidateCutAroundVertex(SeqRemapper& remapper);
  cut::LogicCut getCandidateCut() const {
    return candidate_cut_;
  }

 protected:

  cut::LogicCut candidate_cut_;
  cut::LogicCut worst_cut_;
};

}  // namespace rmp