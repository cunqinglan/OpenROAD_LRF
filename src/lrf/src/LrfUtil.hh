
#pragma once 




namespace sta {
class LibertyCell;
class Sdc;
class Network;
} // namespace sta

namespace lrf {

// Prune illegal equivalent cells and get the violation type.
// Return true if cell are legal
bool isLegalEquivCells(sta::LibertyCell *ori_cell, sta::Sdc *sdc,
                       bool slew_violated, bool cap_violated);
                            
} // namespace lrf