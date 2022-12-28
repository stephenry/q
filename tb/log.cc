//========================================================================== //
// Copyright (c) 2022, Stephen Henry
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//========================================================================== //

#include "log.h"

#include <sstream>
#include <string>
#include <algorithm>
#include <iostream>

#include "tb.h"

void StreamRenderer<bool>::write(std::ostream& os, const bool& b) {
  os << (b ? "1" : "0");
}
void StreamRenderer<const char*>::write(std::ostream& os, const char* msg) {
  os << msg;
}

void StreamRenderer<Level>::write(std::ostream& os, Level l, bool shortform) {
  if (shortform) {
    switch (l) {
#define __declare_level(__level) \
    case Level::__level: os << #__level[0]; break;
    LOG_LEVELS(__declare_level)
#undef __declare_level
    default: os << "U"; break;
    }
  } else {
    switch (l) {
#define __declare_level(__level) \
    case Level::__level: os << #__level; break;
      LOG_LEVELS(__declare_level)
#undef __declare_level
      default: os << "Unknown";
    }
  }
}

void Logger::Context::preamble(std::ostream& os, Level l) const {
  // [(Fatal|Error|Warning|Info|Debug)]{path}: <message>
  StreamRenderer<Level>::write(os, l, true);
  os << PATH_LPAREN;
  if (Globals::kernel) {
    os << Globals::kernel->tb_cycle() << " - ";
  }
  os << s_->path() << PATH_RPAREN << PATH_COLON;
}

void Logger::Context::postamble(std::ostream& os) const {
  os << "\n";
}

Scope::Scope(const std::string& name, Logger* logger, Scope* parent)
  : name_(name), logger_(logger), parent_(parent) {
}

std::string Scope::path() const {
  if (!path_) {
    path_ = render_path();
  }
  return *path_;
}

std::ostream& Scope::os() const { return logger_->os(); }

Scope* Scope::create_child(const std::string& scope_name) {
  children_.emplace_back(
    std::unique_ptr<Scope>(new Scope(scope_name, logger_, this)));
  return children_.back().get();
}

std::string Scope::render_path() const {
  std::vector<std::string> vs;
  vs.push_back(name_);
  Scope* scope = parent_;
  while (scope != nullptr) {
    vs.push_back(scope->name());
    scope = scope->parent_;
  }
  std::string path;
  while (!vs.empty()) {
    if (!path.empty()) {
      path += scope_separator;
    }
    path += vs.back();
    vs.pop_back();
  }
  return path;
}

Logger::Logger() : os_(std::addressof(std::cout)) {}

Scope* Logger::top() {
  if (!parent_scope_) {
    parent_scope_.reset(new Scope("tb", this));
  }
  return parent_scope_.get();
}
