#pragma once

#include <set>
#include <map>
#include <string>
#include <mutex>
#include <vector>
#include <thread>

#include "sta/StaState.hh"
#include "TaskArranger.hh"

namespace lrf {

// Topology violation types
enum class ViolationType {
  FANIN_NOT_VISITED,      // Fanin not visited before this vertex
  SIBLING_ORDER_VIOLATION, // Sibling (same fanout) visited out of order
  CONCURRENT_MODIFICATION  // Same vertex modified by multiple threads
};

struct TopologyViolation {
  ViolationType type;
  std::string vertex_name;
  std::string violation_detail;
  std::thread::id thread_id;
};

// Virtual visitor to check topological traversal correctness
class TopologyChecker {
public:
  TopologyChecker(TaskArranger* arranger, bool enable = true);
  ~TopologyChecker();
  
  // Called when visiting a vertex
  void onVisit(InstVertex* vertex, std::thread::id thread_id);
  
  // Called when a vertex is about to be modified
  void onBeforeModify(InstVertex* vertex, std::thread::id thread_id);
  
  // Called when a vertex modification is committed
  void onAfterModify(InstVertex* vertex, std::thread::id thread_id);
  
  // Check if all fanins have been visited
  bool checkFaninsVisited(InstVertex* vertex);
  
  // Check if siblings are being visited in correct order
  bool checkSiblingOrder(InstVertex* vertex);
  
  // Print violation summary
  void printViolations();
  
  // Get violation count
  size_t getViolationCount() const { return violations_.size(); }
  
  // Clear violations
  void clear();

private:
  TaskArranger* arranger_;
  bool enabled_;
  
  // Track which vertices have been visited
  std::mutex visited_mutex_;
  std::set<VertexId> visited_vertices_;
  
  // Track which vertices are currently being modified
  std::mutex modify_mutex_;
  std::map<VertexId, std::thread::id> vertices_being_modified_;
  
  // Track which vertices have been committed (modified to DB)
  std::mutex committed_mutex_;
  std::set<VertexId> committed_vertices_;
  
  // Store violations
  std::mutex violations_mutex_;
  std::vector<TopologyViolation> violations_;
  
  // Helper functions
  void recordViolation(ViolationType type, InstVertex* vertex, 
                      const std::string& detail, std::thread::id thread_id);
  std::string getVertexName(InstVertex* vertex);
  std::string getThreadIdStr(std::thread::id id);
};

} // namespace lrf
