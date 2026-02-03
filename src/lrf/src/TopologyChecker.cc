#include "TopologyChecker.hh"
#include "sta/Network.hh"
#include <cstdio>
#include <sstream>
#include <iomanip>

namespace lrf {

TopologyChecker::TopologyChecker(TaskArranger* arranger, bool enable)
  : arranger_(arranger), enabled_(enable)
{
}

TopologyChecker::~TopologyChecker()
{
  if (enabled_ && !violations_.empty()) {
    printViolations();
  }
}

std::string TopologyChecker::getVertexName(InstVertex* vertex)
{
  if (!vertex || !vertex->inst()) {
    return "unknown";
  }
  return arranger_->network()->pathName(vertex->inst());
}

std::string TopologyChecker::getThreadIdStr(std::thread::id id)
{
  std::ostringstream oss;
  oss << std::hash<std::thread::id>{}(id);
  return oss.str();
}

void TopologyChecker::recordViolation(ViolationType type, InstVertex* vertex,
                                     const std::string& detail, 
                                     std::thread::id thread_id)
{
  std::lock_guard<std::mutex> lock(violations_mutex_);
  
  TopologyViolation violation;
  violation.type = type;
  violation.vertex_name = getVertexName(vertex);
  violation.violation_detail = detail;
  violation.thread_id = thread_id;
  
  violations_.push_back(violation);
  
  // Print immediately for critical violations
  const char* type_str = "";
  switch (type) {
    case ViolationType::FANIN_NOT_VISITED:
      type_str = "FANIN_NOT_VISITED";
      break;
    case ViolationType::SIBLING_ORDER_VIOLATION:
      type_str = "SIBLING_ORDER_VIOLATION";
      break;
    case ViolationType::CONCURRENT_MODIFICATION:
      type_str = "CONCURRENT_MODIFICATION";
      break;
  }
  
  printf("!!! TOPOLOGY VIOLATION !!!\n");
  printf("Type: %s\n", type_str);
  printf("Vertex: %s\n", violation.vertex_name.c_str());
  printf("Thread: %s\n", getThreadIdStr(thread_id).c_str());
  printf("Detail: %s\n", detail.c_str());
  printf("\n");
  fflush(stdout);
}

void TopologyChecker::onVisit(InstVertex* vertex, std::thread::id thread_id)
{
  if (!enabled_ || !vertex) return;
  
  VertexId vid = vertex->objectIdx();
  
  // Check if fanins have been visited
  if (!checkFaninsVisited(vertex)) {
    // Violation will be recorded in checkFaninsVisited
  }
  
  // Check sibling order
  // if (!checkSiblingOrder(vertex)) {
  //   // Violation will be recorded in checkSiblingOrder
  // }
  
  // Mark as visited
  std::lock_guard<std::mutex> lock(visited_mutex_);
  visited_vertices_.insert(vid);
}

void TopologyChecker::onBeforeModify(InstVertex* vertex, std::thread::id thread_id)
{
  if (!enabled_ || !vertex) return;
  
  VertexId vid = vertex->objectIdx();
  
  std::lock_guard<std::mutex> lock(modify_mutex_);
  
  // Check if already being modified by another thread
  auto it = vertices_being_modified_.find(vid);
  if (it != vertices_being_modified_.end()) {
    if (it->second != thread_id) {
      std::ostringstream detail;
      detail << "Vertex already being modified by thread " 
             << getThreadIdStr(it->second);
      recordViolation(ViolationType::CONCURRENT_MODIFICATION, 
                     vertex, detail.str(), thread_id);
    }
  }
  
  vertices_being_modified_[vid] = thread_id;
}

void TopologyChecker::onAfterModify(InstVertex* vertex, std::thread::id thread_id)
{
  if (!enabled_ || !vertex) return;
  
  VertexId vid = vertex->objectIdx();
  
  {
    std::lock_guard<std::mutex> lock(modify_mutex_);
    vertices_being_modified_.erase(vid);
  }
  
  {
    std::lock_guard<std::mutex> lock(committed_mutex_);
    committed_vertices_.insert(vid);
  }
}

bool TopologyChecker::checkFaninsVisited(InstVertex* vertex)
{
  if (!enabled_ || !vertex) return true;
  
  std::lock_guard<std::mutex> lock(visited_mutex_);
  
  // Iterate through all fanin edges
  InstVertexInEdgeIterator edge_iter(vertex, arranger_);
  bool all_visited = true;
  
  while (edge_iter.hasNext()) {
    EdgeId edge_id = edge_iter.next();
    InstEdge* edge = arranger_->edge(edge_id);
    VertexId fanin_vid = edge->from();
    InstVertex* fanin_vertex = arranger_->vertex(fanin_vid);
    if (fanin_vertex->type() == VertexType::TOP || 
        fanin_vertex->type() == VertexType::SEQUENTIAL) {
      // Skip TOP vertices
      continue;
    }
    
    if (visited_vertices_.find(fanin_vid) == visited_vertices_.end()) {
      // Fanin not visited yet - this is a violation
      InstVertex* fanin_vertex = arranger_->vertex(fanin_vid);
      std::ostringstream detail;
      detail << "Fanin vertex '" << getVertexName(fanin_vertex) 
             << "' (id=" << fanin_vid << ") not visited before this vertex";
      
      recordViolation(ViolationType::FANIN_NOT_VISITED, 
                     vertex, detail.str(), std::this_thread::get_id());
      all_visited = false;
    }
  }
  
  return all_visited;
}

bool TopologyChecker::checkSiblingOrder(InstVertex* vertex)
{
  if (!enabled_ || !vertex) return true;
  
  // Get all sibling vertices (vertices with same fanin)
  // This is a simplified check - we check if any sibling fanout
  // of the same driver has been modified while this one is being visited
  
  std::lock_guard<std::mutex> vlock(visited_mutex_);
  std::lock_guard<std::mutex> clock(committed_mutex_);
  
  VertexId vid = vertex->objectIdx();
  bool order_ok = true;
  
  // Iterate through fanins
  InstVertexInEdgeIterator edge_iter(vertex, arranger_);
  while (edge_iter.hasNext()) {
    EdgeId edge_id = edge_iter.next();
    InstEdge* edge = arranger_->edge(edge_id);
    VertexId fanin_vid = edge->from();
    InstVertex* fanin_vertex = arranger_->vertex(fanin_vid);
    
    // Check all fanouts of this fanin (siblings)
    InstVertexOutEdgeIterator sibling_iter(fanin_vertex, arranger_);
    while (sibling_iter.hasNext()) {
      EdgeId sibling_edge_id = sibling_iter.next();
      InstEdge* sibling_edge = arranger_->edge(sibling_edge_id);
      VertexId sibling_vid = sibling_edge->to();
      
      if (sibling_vid == vid) continue; // Skip self
      
      // Check if sibling has been committed but this hasn't been visited
      if (committed_vertices_.find(sibling_vid) != committed_vertices_.end() &&
          visited_vertices_.find(vid) == visited_vertices_.end()) {
        
        InstVertex* sibling_vertex = arranger_->vertex(sibling_vid);
        std::ostringstream detail;
        detail << "Sibling vertex '" << getVertexName(sibling_vertex)
               << "' (id=" << sibling_vid << ") was committed before this vertex was visited. "
               << "Both are fanouts of '" << getVertexName(fanin_vertex) << "'";
        
        recordViolation(ViolationType::SIBLING_ORDER_VIOLATION,
                       vertex, detail.str(), std::this_thread::get_id());
        order_ok = false;
      }
    }
  }
  
  return order_ok;
}

void TopologyChecker::printViolations()
{
  std::lock_guard<std::mutex> lock(violations_mutex_);
  
  if (violations_.empty()) {
    printf("\n=== TOPOLOGY CHECK: No violations detected ===\n");
  } else {
    printf("\n=== TOPOLOGY VIOLATIONS DETECTED: %zu violations ===\n", 
           violations_.size());
    
    size_t fanin_count = 0;
    size_t sibling_count = 0;
    size_t concurrent_count = 0;
    
    for (const auto& v : violations_) {
      switch (v.type) {
        case ViolationType::FANIN_NOT_VISITED:
          fanin_count++;
          break;
        case ViolationType::SIBLING_ORDER_VIOLATION:
          sibling_count++;
          break;
        case ViolationType::CONCURRENT_MODIFICATION:
          concurrent_count++;
          break;
      }
    }
    
    printf("  - Fanin not visited: %zu\n", fanin_count);
    printf("  - Sibling order violations: %zu\n", sibling_count);
    printf("  - Concurrent modifications: %zu\n", concurrent_count);
    printf("\n");
  }
  
  fflush(stdout);
}

void TopologyChecker::clear()
{
  std::lock_guard<std::mutex> vlock(visited_mutex_);
  std::lock_guard<std::mutex> mlock(modify_mutex_);
  std::lock_guard<std::mutex> clock(committed_mutex_);
  std::lock_guard<std::mutex> violock(violations_mutex_);
  
  visited_vertices_.clear();
  vertices_being_modified_.clear();
  committed_vertices_.clear();
  violations_.clear();
}

} // namespace lrf
