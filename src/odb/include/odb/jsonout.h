#pragma once

#include <memory>

#include "odb/odb.h"

namespace utl {
class Logger;
}

namespace odb {

class dbInst;
class dbNet;
class dbBlock;

class JsonOut
{
 public:
  JsonOut(utl::Logger* logger);
  ~JsonOut();

  void selectNet(dbNet* net);
  void selectInst(dbInst* inst);

  bool writeBlock(dbBlock* block, const char* json_file);

 private:
  class Impl;
  std::unique_ptr<Impl> writer_;
};



}  // namespace odb