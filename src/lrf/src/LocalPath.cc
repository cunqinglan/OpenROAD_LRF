

#include "sta/Search.hh"
#include "sta/Path.hh"
#include "search/Tag.hh"
#include "sta/PathAnalysisPt.hh"

#include "LocalSearch.hh"
#include "PtGraph.hh"


namespace lrf {
PtVertexPathIterator::PtVertexPathIterator(PtVertex &pt_vertex,
                                 const sta::StaState *sta)
  : sta::VertexPathIterator(pt_vertex.vertex(), sta)
{
  paths_ = pt_vertex.paths();
}

PtVertexPathIterator::~PtVertexPathIterator()
{
}

void 
PtVertexPathIterator::findNext()
{
  while (path_index_ < path_count_) {
    Path *path = &paths_[path_index_++];
    // Value filter should be false here!
    if (filtered_) {
      const Tag *tag = path->tag(search_);
      if ((rf_ == nullptr
           || tag->rfIndex() == rf_->index())
          && (path_ap_ == nullptr
              || tag->pathAPIndex() == path_ap_->index())
          && (min_max_ == nullptr
              || tag->pathAnalysisPt(search_)->pathMinMax() == min_max_)) {
        next_ = path;
        return;
      }
    }
    else {
      next_ = path;
      return;
    }
  }
  next_ = nullptr;
}





} // namespace lrf