#pragma once

#include <string>

namespace fly {

// Find the directory holding cns.bin.
//
// The default "data/bin" only resolves when the working directory happens to
// be the project root, which it is not when the executable is launched from
// its build directory or double-clicked from a file manager. This searches the
// obvious places instead: the hint as given, then upward from the working
// directory, then upward from the executable's own location.
//
// Throws std::runtime_error naming every path it tried, so a failure says what
// to do rather than just that something was missing.
//
// `argv0` may be null; the executable-relative search is then skipped.
std::string findDataDir(const std::string& hint, const char* argv0);

}  // namespace fly
