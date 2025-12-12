

#include "sta/Search.hh"
#include "sta/Path.hh"
#include "search/Tag.hh"
#include "sta/PathAnalysisPt.hh"
#include "search/TagGroup.hh"

#include "LocalSearch.hh"
#include "PtGraph.hh"


namespace lrf {
PtVertexPathIterator::PtVertexPathIterator(PtVertex &pt_vertex,
                                 const sta::StaState *sta)
  : search_(sta->search()),
    filtered_(false),
    rf_(nullptr),
    path_ap_(nullptr),
    min_max_(nullptr),
    paths_(pt_vertex.paths()),
    path_count_(0),
    path_index_(0),
    next_(nullptr)
{

  sta::TagGroup *tag_group = search_->tagGroup(pt_vertex.tagGroupIndex());
  if (tag_group) {
    path_count_ = tag_group->pathCount();
    findNext();
  }
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


bool
PtVertexPathIterator::hasNext()
{
  return next_ != nullptr;
}

Path *
PtVertexPathIterator::next()
{
  Path *path = next_;
  findNext();
  return path;
}



} // namespace lrf