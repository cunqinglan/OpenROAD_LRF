


#include "GraphBase.hh"


namespace lrf {
VertexBase::VertexBase()
{
  object_idx_ = object_id_null;
  in_edges_ = edge_id_null;
  out_edges_ = edge_id_null;
}

EdgeBase::EdgeBase()
{
  object_idx_ = object_id_null;
  from_ = vertex_idx_null;
  to_ = vertex_idx_null;
  vertex_in_link_ = edge_id_null;
  vertex_out_prev_ = edge_id_null;
  vertex_out_next_ = edge_id_null;
}

void 
EdgeBase::init(VertexId from,
               VertexId to)
{
  from_ = from;
  to_ = to;
}

// template<typename VertexT, typename EdgeT>
// GraphBase<VertexT, EdgeT>::GraphBase()
//   : vertices_(),
//     edges_() 
// {
// }

// template<typename VertexT, typename EdgeT>
// GraphBase<VertexT, EdgeT>::~GraphBase() 
// {
//   vertices_.clear();
//   edges_.clear();
// }



} // namespace lrf