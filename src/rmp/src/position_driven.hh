#include <limits>
#include <vector>
#include <string>

#include "cut/logic_cut.h"
#include "rmp/RemapConfig.hh"
#include "Strategy.hh"


namespace rmp {

// Result of evaluating a single mapping solution in a child process.
struct SolutionEvalResult {
  int solution_index;
  abc::Map_MappingSolution_t* pSolution;
  sta::Slack slack;
  std::string log;
  bool success;
};

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

  // Endpoint selection parameters:
  //   percentage    >= 0  : fix that % of all endpoints (min 1); overrides others
  //   max_percentage >= 0  : cap endpoint count at this % of all endpoints
  //   slack_threshold      : select endpoints with slack < threshold (with max_percentage)
  //   Defaults (-1 / FLT_MAX) mean "not set"; if nothing set, fix only the worst endpoint.
  void remap(SeqRemapper& remapper,
             float percentage = -1.0f,
             float max_percentage = -1.0f,
             float slack_threshold = std::numeric_limits<float>::max(),
             bool run_detailed_placement = false,
             bool verbose = false,
             RemapConfig config = {});

  sta::Slack evaluateSolution(abc::Map_MappingSolution_t* pSolution,
                              abc::Map_Man_t* pMan,
                              abc::Abc_Ntk_t* pOriginalNetwork,
                              cut::LogicCut& candidate_cut,
                              SeqRemapper& remapper);

  sta::PinSet getCutFanoutEndpoints(
      cut::LogicCut& candidate_cut,
      sta::dbSta* sta,
      sta::dbNetwork* network);

  sta::Slack getWorstSlackFromEndpoints(
      const sta::PinSet& fanout_endpoints,
      sta::dbSta* sta);

  // Phase 2: Re-evaluate a single solution with localized repairSetup
  // in the parent process. Uses ECO journaling so changes can be undone.
  // Returns post-repair worst slack, or lowest() on failure.
  // After return, the ECO is on the stack (caller must undoEco to revert).
  sta::Slack reEvaluateWithRepair(
      abc::Map_MappingSolution_t* pSolution,
      abc::Map_Man_t* pMan,
      abc::Abc_Ntk_t* pOriginalNetwork,
      cut::LogicCut& candidate_cut,
      SeqRemapper& remapper);

  // Fork-evaluate a range of solutions [iStart, iEnd) in parallel.
  // Returns results for each solution including slack and log output.
  std::vector<SolutionEvalResult> forkEvaluateSolutions(
      abc::Map_Man_t* map_man,
      abc::Abc_Ntk_t* logic_network,
      cut::LogicCut& candidate_cut,
      SeqRemapper& remapper,
      int iStart,
      int iEnd);

  //void positionDrivenRemap (SeqRemapper& remapper);
  sta::Vertex* getFarthestOutputVertex(
      SeqRemapper& remapper);
  std::vector<sta::Vertex*> getWorstVertices(
      SeqRemapper& remapper,
      float percentage = -1.0f,
      float max_percentage = -1.0f,
      float slack_threshold = std::numeric_limits<float>::max());
  std::vector<sta::Vertex*> getWorstVerticesForEndpoint(
      SeqRemapper& remapper,
      sta::Vertex* endpoint);
  bool remapOneCut(
      SeqRemapper& remapper,
      std::vector<sta::Vertex*>& worst_vertices);

  // Build a candidate cut for gate cloning when the extracted cut is too large.
  // Clones bad_instance, redirects up to max_cut_instances of its fanout
  // instances (sorted worst-slack first) to the clone's output, and returns
  // a LogicCut over {clone} ∪ {selected fanouts}.
  // Returns an empty cut ({},{},{}) if gate cloning is not applicable.
  cut::LogicCut buildGateCloneCut(sta::Instance* bad_instance,
                                  SeqRemapper& remapper,
                                  size_t max_cut_instances);
  void extractCandidateCutAroundVertex(SeqRemapper& remapper);
  cut::LogicCut getCandidateCut() const {
    return candidate_cut_;
  }

 protected:

  cut::LogicCut candidate_cut_;
  cut::LogicCut worst_cut_;
  bool verbose_ = false;
  RemapConfig config_;
};

}  // namespace rmp