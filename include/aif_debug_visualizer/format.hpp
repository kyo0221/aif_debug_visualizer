#pragma once
#include <cstdio>
#include <string>

namespace aif_debug_visualizer {

constexpr const char * kDash = "\xE2\x80\x94";

template <typename... A>
std::string fmt(const char * f, A... a)
{
  char b[160];
  std::snprintf(b, sizeof(b), f, a...);
  return b;
}

}  // namespace aif_debug_visualizer
