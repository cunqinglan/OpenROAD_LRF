#include "cut/logic_cut.h"
#include "Strategy.hh"

class PositionDrivenStrategy : public ExtractLocalWindow
{
 public:
  PositionDrivenStrategy(utl::Logger *logger) : Strategy(logger) {}
    ~PositionDrivenStrategy() override = default;

  void setCandidateCut(const cut::LogicCut &cut) {
    candidate_cut_ = cut;
  }
  void setWorstCut(const cut::LogicCut &cut) {
    worst_cut_ = cut;
  }
  void 

  
  //void positionDrivenRemap (SeqRemapper& remapper);
  sta::Vertex* getFarthestOutputVertex(
      SeqRemapper& remapper);
  sta::Vertex* getWorstVertex(
      SeqRemapper& remapper);
  void extractCandidateCutAroundVertex(SeqRemapper& remapper);
  cut::LogitCut getCandidateCut() const {
    return candidate_cut_;
  }

 protected:

  cut::LogicCut candidate_cut_;
  cut::LogicCut worst_cut_;
};