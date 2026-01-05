// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "odb/jsonout.h"

#include <cassert>
#include <cstdio>
#include <memory>

#include "jsonout_impl.h"
#include "odb/db.h"

namespace odb {

JsonOut::JsonOut(utl::Logger* logger)
    : writer_(std::make_unique<Impl>(logger))
{}

JsonOut::~JsonOut() = default;
void JsonOut::selectNet(dbNet* net)
{
  writer_->selectNet(net);
}

void JsonOut::selectInst(dbInst* inst)
{
  writer_->selectInst(inst);
}

bool JsonOut::writeBlock(dbBlock* block, const char* json_file)
{
  return writer_->writeBlock(block, json_file);
}

}  // namespace odb
