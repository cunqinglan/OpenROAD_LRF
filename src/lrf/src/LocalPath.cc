

#include "sta/Search.hh"
#include "sta/Path.hh"
#include "search/Tag.hh"
#include "sta/Scene.hh"
#include "search/TagGroup.hh"

#include "LocalSearch.hh"
#include "PtGraph.hh"


namespace lrf {
PtVertexPathIterator::PtVertexPathIterator(PtVertex &pt_vertex,
                                 const sta::StaState *sta)
  : search_(sta->search()),
    filtered_(false),
    rf_(nullptr),
    min_max_(nullptr),
    paths_(pt_vertex.paths()),
    path_count_(0),
    path_index_(0),
    next_(nullptr)
{
  // If the tag group index is valid and paths are allocated, initialize path count.
  // paths_ may be nullptr for virtual vertices whose localSetVertexArrivals returned
  // early (e.g. findExistingTagGroup returned null). Guard against dereferencing null.
  int tag_group_index = pt_vertex.tagGroupIndex();
  if (tag_group_index != tag_group_index_max) {
    if (paths_ == nullptr) {
      printf("PtVertexPathIterator: vertex objectIdx=%u has tagGroupIndex=%d but paths_==nullptr;"
             " treating as empty (virtual vertex not yet initialized).\n",
             pt_vertex.objectIdx(), tag_group_index);
      fflush(stdout);
      // Leave path_count_=0 so findNext() is not called — nothing to iterate.
    } else {
      sta::TagGroup *tag_group = search_->tagGroup(pt_vertex.tagGroupIndex());
      if (tag_group == nullptr) {
        printf("PtVertexPathIterator: vertex objectIdx=%u tagGroupIndex=%d but tagGroup is null;"
               " treating as empty.\n",
               pt_vertex.objectIdx(), tag_group_index);
        fflush(stdout);
      } else {
        path_count_ = tag_group->pathCount();
        findNext();
      }
    }
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
          && (min_max_ == nullptr
              || tag->minMax() == min_max_)) {
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