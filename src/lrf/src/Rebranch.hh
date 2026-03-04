#pragma once

#include "est/SteinerTree.h"
#include <unordered_map>

namespace sta {
class Pin;
class dbSta;
}


namespace lrf {


// Timing-aware re-branching (topology-only).
//
// This utility rewires a subset of sink connections to jump upstream in the
// Steiner tree topology.
//
// Inputs:
//  - in_tree: an `est::SteinerTree` created by `est::EstimateParasitics`.
//  - sink_criticality: per-sink criticality C(si). The map keys are load pins.
//    Values are unnormalized; the function normalizes internally.
//  - max_rebranch_ratio: max fraction of sinks to rebranch (default 0.2).
//
// Output:
//  - A newly allocated `est::SteinerTree*` that reflects the re-branched
//    topology. Ownership is transferred to the caller.
//
// NOTE: This is a topology-only implementation and does not attempt to
// merge/track geometric overlap of Manhattan segments.
// Rebranch the Steiner tree in-place.
//
// Returns true if at least one sink was rebranched, false if it was a no-op
// (for example, sinks.empty() or all candidates rejected).
bool rebranchTopologyInPlace(
	est::SteinerTree* tree,
	sta::dbSta* sta,
	const std::unordered_map<const sta::Pin*, double>& sink_criticality,
	double max_rebranch_ratio = 0.2);

// APIs for rebranching and sink criticality computation
// Compute sink criticality as sum of all negative slacks
// If corner is nullptr, use default corner; otherwise use specified corner
std::unordered_map<const sta::Pin*, double> computeSinkCriticality(
	const sta::Pin* drvr_pin,
	sta::Corner* corner,
	sta::dbSta* sta);

// Print formatted sink criticality information
void printSinkCriticality(
	const sta::Pin* drvr_pin,
	const std::unordered_map<const sta::Pin*, double>& sink_criticality,
	sta::dbSta* sta);


// Convenience API: create a Steiner tree for drvr_pin and return the re-branched
// result.
//
// This is the intended top-level entry point for lrf usage so the caller does
// not need to separately build the initial tree then rebranch it.
//
// Returns nullptr if the net is unplaced, has <2 pins, or tree build fails.
est::SteinerTree* makeRebranchedSteinerTree(
	const sta::Pin* drvr_pin,
	sta::dbSta* sta,
	const std::unordered_map<const sta::Pin*, double>& sink_criticality,
	double max_rebranch_ratio = 0.2);



} // namespace lrf