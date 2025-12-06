#pragma once

// #include 

namespace rsz
{
class Resizer;
}  // namespace rsz


namespace lrf
{

class  TestLrf
{
public:
  void testLocalDelayCompute(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);
};

}  // namespace lrf