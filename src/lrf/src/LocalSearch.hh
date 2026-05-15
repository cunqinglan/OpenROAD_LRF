#pragma once

#include "sta/StaState.hh"
#include "sta/Search.hh"
#include "sta/GraphClass.hh"
#include "sta/LibertyClass.hh"
#include "sta/NetworkClass.hh"
#include "sta/Delay.hh"

#include <stdexcept>

namespace sta
{
class StaState;
class TagGroup;
class ArrivalVisitor;
class Graph;
class RiseFall;
class Path;
class TimingArc;
class MinMax;
class Tag;
class TagGroupBldr;
class SearchPred;
} // namespace sta

namespace lrf
{
class PtGraph;
class PtVertex;
class PtEdge;

using namespace sta;

class CombinationArrivalVisitor
{
public:
  CombinationArrivalVisitor(StaState *state, PtGraph *pt_graph);
  ~CombinationArrivalVisitor();

  void seedLocalRootArrivals(PtVertex &pt_vertex);
  void findVertexArrival(PtVertex &pt_vertex);

  void visit(PtVertex &pt_vertex);
  void visitEdge(PtVertex &from_pt_vertex, 
                 PtEdge &edge, 
                 PtVertex &to_pt_vertex);
  bool visitArc(PtVertex &from_pt_vertex,
                const RiseFall *from_rf,
                Path *from_path,
                PtEdge &edge,
                TimingArc *arc,
                PtVertex &to_pt_vertex,
                const MinMax *min_max);
  bool visitFromToPath(const Pin *from_pin,
                              Vertex *from_vertex,
                              const RiseFall *from_rf,
                              Path *from_path,
                              Arrival from_arrival,
                      Edge *edge,
                      TimingArc *arc,
                      Delay arc_delay,
                      Vertex *to_vertex,
                      const RiseFall *to_rf,
                      Tag *to_tag,
                      Arrival to_arrival,
                      const MinMax *min_max);
protected:
  StaState *sta_;
  PtGraph *pt_graph_;
};

class LocalPathVisitor : public PathVisitor
{
public:
  LocalPathVisitor(StaState *state, PtGraph *pt_graph);
  LocalPathVisitor(StaState *state, PtGraph *pt_graph, const std::string &debug_label);
  virtual ~LocalPathVisitor();

  virtual VertexVisitor *copy() const override;
  void localVisitFaninPaths(PtVertex &to_pt_vertex);
  void localVisitFanoutPaths(PtVertex &from_pt_vertex);
  bool localVisitEdge(PtVertex &from_pt_vertex, 
                       PtEdge &edge, 
                       PtVertex &to_pt_vertex);
  bool localVisitArc(PtVertex &from_pt_vertex,
                      const RiseFall *from_rf,
                      Path *from_path,
                      PtEdge &edge,
                      TimingArc *arc,
                      PtVertex &to_pt_vertex,
                      const MinMax *min_max);
  virtual bool localVisitFromPath(const Pin *from_pin,
                          PtVertex &from_pt_vertex,
                          const RiseFall *from_rf,
                          Path *from_path,
                          PtEdge &edge,
                          TimingArc *arc,
                          const Pin *to_pin,
                          PtVertex &to_pt_vertex,
                          const RiseFall *to_rf,
                          const MinMax *min_max);
  virtual bool localVisitFromToPath(
                PtVertex &from_pt_vertex,
                const RiseFall *from_rf,
                Tag *from_tag,
                Path *from_path,
                const Arrival &from_arrival,
                PtEdge &pt_edge,
                TimingArc *arc,
                ArcDelay arc_delay,
                PtVertex &to_pt_vertex,
                const RiseFall *to_rf,
                Tag *to_tag,
                Arrival &to_arrival,
                const MinMax *min_max) = 0;
  // Just delete this implementation
  virtual bool visitFromToPath(const Pin *from_pin,
			       Vertex *from_vertex,
			       const RiseFall *from_rf,
			       Tag *from_tag,
			       Path *from_path,
                              const Arrival &from_arrival,
			       Edge *edge,
			       TimingArc *arc,
			       ArcDelay arc_delay,
			       Vertex *to_vertex,
			       const RiseFall *to_rf,
			       Tag *to_tag,
			       Arrival &to_arrival,
			       const MinMax *min_max) override { return false; }

protected:
  StaState *sta_;
  PtGraph *pt_graph_;
  std::string debug_label_;
};


class LocalArrivalVisitor : public LocalPathVisitor
{
public:
  LocalArrivalVisitor(StaState *state, PtGraph *pt_graph, const std::string &debug_label);
  LocalArrivalVisitor(StaState *state, PtGraph *pt_graph);
  ~LocalArrivalVisitor();
  void init();

  virtual void visit(Vertex *vertex) override;

  // Find arrivals in the local graph
  void findLocalArrivals();
  // Copy arrivals of input vertices this searcher
  void findArrivalsSeed();
  // visit given vertex, compute its arrival
  // It's actually the visit function of ArrivalVisitor
  void findVertexArrival(VertexId vertex_id);
  void findVertexArrival(PtVertex &vertex);
  // Seed arrival for root vertices (no fanin)
  void seedLocalRootArrivals(PtVertex &pt_vertex);
  virtual bool localVisitFromToPath(
                PtVertex &from_pt_vertex,
                const RiseFall *from_rf,
                Tag *from_tag,
                Path *from_path,
                const Arrival &from_arrival,
                PtEdge &pt_edge,
                TimingArc *arc,
                ArcDelay arc_delay,
                PtVertex &to_pt_vertex,
                const RiseFall *to_rf,
                Tag *to_tag,
                Arrival &to_arrival,
                const MinMax *min_max) override;
  void printArrivals();

protected:
  void localSetVertexArrivals(PtVertex &vertex, TagGroupBldr *tag_bldr);
  void findVirtualVertexArrival(PtVertex &pt_vertex);
  void init0();

  bool always_to_endpoints_;
  bool always_save_prev_paths_;
  TagGroupBldr *tag_bldr_;
  bool has_fanin_one_;
};

class LocalRequiredCmp
{
public:
  LocalRequiredCmp();
  void requiredsInit(PtVertex &pt_vertex,
		     const StaState *sta);
  void requiredSet(size_t path_index,
		   Required &required,
		   const MinMax *min_max,
		   const StaState *sta);
  // Return true if the requireds changed.
  bool requiredsSave(PtVertex &pt_vertex,
		     const StaState *sta);
  Required required(size_t path_index);

protected:
  ArrivalSeq requireds_;
  // Since we update requireds for all swapping, it is not necessary
  bool have_requireds_;
};


class LocalRequiredVisitor : public LocalPathVisitor
{
public:
  LocalRequiredVisitor(StaState *state, PtGraph *pt_graph);
  ~LocalRequiredVisitor();

  virtual void visit(Vertex *vertex) override;

  // Find requireds in the local graph
  void findLocalRequireds();
  // visit given vertex, compute its required
  void findVertexRequired(VertexId vertex_id);
  void findVertexRequired(PtVertex &vertex);

  virtual bool localVisitFromToPath(
                PtVertex &from_pt_vertex,
                const RiseFall *from_rf,
                Tag *from_tag,
                Path *from_path,
                const Arrival &from_arrival,
                PtEdge &pt_edge,
                TimingArc *arc,
                ArcDelay arc_delay,
                PtVertex &to_pt_vertex,
                const RiseFall *to_rf,
                Tag *to_tag,
                Arrival &to_arrival,
                const MinMax *min_max) override;
  void printRequireds();
protected:
  void seedLocalRootRequireds(PtVertex &pt_vertex);

  // It is a comparator and storage for requireds during the search
  LocalRequiredCmp *required_cmp_;

};


class PtVertexPathIterator: public Iterator<Path*>
{
public:
  PtVertexPathIterator(PtVertex &pt_vertex,
           const sta::StaState *sta);
  ~PtVertexPathIterator();
  virtual bool hasNext();
  virtual Path *next();
  size_t pathIndex() const;

protected:
  void findNext();

  const Search *search_;
  bool filtered_;
  const RiseFall *rf_;
  const MinMax *min_max_;
  Path *paths_;
  size_t path_count_;
  size_t path_index_;
  Path *next_;
};

size_t ptPathIndex(PtVertex &pt_vertex, Path *path);



















} // namespace lrf