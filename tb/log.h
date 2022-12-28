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

#ifndef Q_TB_LOG_H
#define Q_TB_LOG_H

#include <iostream>
#include <vector>
#include <memory>
#include <ostream>
#include <sstream>
#include <optional>
#include <ios>
#include "verilated.h"

#define MACRO_BEGIN    do {
#define MACRO_END      } while (false)

#define Q_ASSERT(__lg, __cond) \
  MACRO_BEGIN \
  if (!(__cond)) { \
    ++::tb::Sim::errors; \
    if (__lg) { \
      __lg->Fatal("Assertion failed: ", #__cond); \
    } \
  } \
  MACRO_END

#define Q_LOG(__lg, __level, __msg) \
  Q_LOG_IF(__lg, true, __level, __msg)

#define Q_LOG_IF(__lg, __cond, __level, ...) \
  MACRO_BEGIN \
  if ((__lg) && (__cond)) { \
    __lg->__level(__VA_ARGS__); \
  } \
  switch (::tb::Level::__level) { \
  case ::tb::Level::Warning: ++::tb::Sim::warnings; break; \
  case ::tb::Level::Error:   ++::tb::Sim::errors; break; \
  default: break; \
  } \
  MACRO_END

#define LOG_LEVELS(__func)  \
  __func(Debug)\
  __func(Info)\
  __func(Warning)\
  __func(Error)\
  __func(Fatal)

enum class Level {
#define __declare_level(__level) __level,
  LOG_LEVELS(__declare_level)
#undef __declare_level
};

class Logger;

template<typename T>
struct AsHex {
  explicit AsHex(const T& t, bool showbase) : t(t), showbase(showbase) {}
  const T& t;
  const bool showbase;
};

template<typename T>
struct AsDec {
  explicit AsDec(const T& t, bool showbase) : t(t), showbase(showbase) {}
  const T& t;
  const bool showbase;
};

struct SetFlags {
  explicit SetFlags(std::ostream& os, std::ios_base::fmtflags flags_new)
    : os_(os) {
    flags_old_ = os_.flags(flags_new);
  }
  ~SetFlags() {
    os_.flags(flags_old_);
  }

private:
  std::ostream& os_;
  std::ios_base::fmtflags flags_old_;
};

template<typename T>
struct StreamRenderer;

template<>
struct StreamRenderer<bool> {
  static void write(std::ostream& os, const bool& t);
};

template<>
struct StreamRenderer<const char*> {
  static void write(std::ostream& os, const char* msg);
};

template<>
struct StreamRenderer<Level> {
  static void write(std::ostream& os, Level l, bool shortform = false);
};

template<typename T>
struct StreamRenderer<AsHex<T>> {
  static void write(std::ostream& os, const AsHex<T>& h) {
    std::ios_base::fmtflags flags{std::ios_base::hex};
    if (h.showbase) { flags |= std::ios_base::showbase; }
    SetFlags sf(os, flags);
    StreamRenderer<T>::write(os, h.t);
  }
};

template<typename T>
struct StreamRenderer<AsDec<T>> {
  static void write(std::ostream& os, const AsDec<T>& d) {
    std::ios_base::fmtflags flags{std::ios_base::dec};
    if (d.showbase) { flags |= std::ios_base::showbase; }
    SetFlags sf(os, flags);
    StreamRenderer<T>::write(os, d.t);
  }
};

template<>
struct StreamRenderer<vluint8_t> {
  static void write(std::ostream& os, vluint8_t c) {
    os << static_cast<vluint32_t>(c);
  }
};

#define GENERIC_TYPES(__func) \
  __func(vlsint64_t) \
  __func(vluint32_t) \
  __func(int) \
  __func(std::size_t) \
  __func(std::string) \
  __func(std::string_view)

#define __declare_handler(__type) \
template<> \
struct StreamRenderer<__type> { \
  static void write(std::ostream& os, const __type& t) { \
    os << t; \
  } \
};
GENERIC_TYPES(__declare_handler)
#undef __declare_handler

class RecordRenderer {
  static constexpr const char* LPAREN = "{";
  static constexpr const char* RPAREN = "}";
public:
  explicit RecordRenderer(std::ostream& os, const std::string& type = "unk")
    : os_(os) {
    os_ << type << LPAREN;
  }

  ~RecordRenderer() {
    if (!finalized_) finalize();
  }

  template<typename T>
  void add(std::string_view k, const T& t) {
    if (finalized_) return;
    preamble(k);
    writekey(t);
  }

private:
  void preamble(std::string_view k) {
    if (entries_n_++) os_ << ", ";
    os_ << k;
    os_ << ":";    
  }
  
  template<typename T>
  void writekey(T& t) {
    StreamRenderer<std::decay_t<T>>::write(os_, t);
  }

  void finalize() {
    os_ << RPAREN;
    finalized_ = true;
  }
  //! Record has been serialized.
  bool finalized_{false};
  //! Number of previously rendered key/value pairs.
  std::size_t entries_n_{0};
  //! Output stream.
  std::ostream& os_;
};

class Scope {
  friend class Logger;

  static constexpr const char* scope_separator = ".";

  explicit Scope(
    const std::string& name, Logger* logger, Scope* parent = nullptr);
public:
  //! Current scope name
  std::string name() { return name_; }
  //! Current scope path
  std::string path() const;

#define __declare_message(__level) \
  template<typename ...Ts> \
  void __level(Ts&& ...ts) const { \
    write(Level::__level, std::forward<Ts>(ts)...); \
  }
  LOG_LEVELS(__declare_message)
#undef __declare_message

  Scope* create_child(const std::string& scope_name);

private:
  template<typename ...Ts>
  void write(Level l, Ts&& ...ts) const;

  //!
  std::string render_path() const;
  //! Name of current scope.
  std::string name_;
  //! Pointer to parent scope (nullptr if root scope)
  Scope* parent_{nullptr};
  //! Owning pointer to child logger scopes.
  std::vector<std::unique_ptr<Scope>> children_;
  //! Path of scope within logger hierarchy.
  mutable std::optional<std::string> path_;
  //! Parent Logger instance.
  Logger* logger_{nullptr};
};

class Logger {
  friend class Scope;
  friend class Context;
public:
  class Context {
    friend class Logger;

    static constexpr const char* PATH_LPAREN = "{";
    static constexpr const char* PATH_RPAREN = "}";
    static constexpr const char* PATH_COLON = ": ";

    explicit Context(const Scope* s, Logger* logger)
     : s_(s), logger_(logger)
    {}
  public:
    template<typename ...Ts>
    void write(Level l, Ts&& ...ts) {
      if (logger_->get_log_level() <= l) {
        std::ostream& os{logger_->os()};
        preamble(os, l);
        (StreamRenderer<std::decay_t<Ts>>::write(os, std::forward<Ts>(ts)), ...);
        postamble(os);
      }
    }

  private:
    void preamble(std::ostream& os, Level l) const;
    void postamble(std::ostream& os) const;

    const Scope* s_;
    Logger* logger_;

  };

  explicit Logger();

  template<typename ...Ts>
  void write(Ts&& ...ts) {
    (StreamRenderer<std::decay_t<Ts>>::write(os(), std::forward<Ts>(ts)), ...);
    os() << "\n";
  }

  Scope* top();

  Level get_log_level() const { return log_level_; }
  
  void set_log_level(Level log_level) { log_level_ = log_level; }

  void set_os(std::ostream* os) { os_ = os; }

  Context create_context(const Scope* s) { return Context{s, this}; }

private:
  //! 
  std::ostream& os() const { return *os_; }
  //!
  std::unique_ptr<Scope> parent_scope_;
  //! Output logging stream.
  std::ostream* os_;
  //! Current log level (everything above is traced)
  Level log_level_{Level::Debug};
};

template<typename ...Ts>
void Scope::write(Level l, Ts&& ...ts) const {
  auto context{logger_->create_context(this)};
  context.write(l, std::forward<Ts>(ts)...);
}

#endif
