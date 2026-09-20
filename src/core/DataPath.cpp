#include "DataPath.h"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace fly {
namespace {

namespace fs = std::filesystem;

constexpr int kMaxLevelsUp = 6;

bool holdsConnectome(const fs::path& dir) {
    std::error_code ec;
    return fs::exists(dir / "cns.bin", ec);
}

// Walk up from `start`, checking both the directory itself and a data/bin
// beneath it, so this works whether you are standing in the project root, in
// build/Release, or anywhere between.
bool searchUpward(fs::path start, const fs::path& suffix,
                  std::vector<std::string>& tried, std::string& found) {
    std::error_code ec;
    start = fs::absolute(start, ec);
    for (int i = 0; i <= kMaxLevelsUp; ++i) {
        const fs::path candidate = start / suffix;
        tried.push_back(candidate.string());
        if (holdsConnectome(candidate)) {
            found = candidate.string();
            return true;
        }
        if (!start.has_parent_path() || start.parent_path() == start) break;
        start = start.parent_path();
    }
    return false;
}

}  // namespace

std::string findDataDir(const std::string& hint, const char* argv0) {
    std::vector<std::string> tried;
    std::string found;
    std::error_code ec;

    // An explicit --data that works is always honoured first.
    tried.push_back(fs::absolute(hint, ec).string());
    if (holdsConnectome(hint)) return hint;

    const fs::path hintPath(hint);
    // Only search for the hint by name if it is relative; an absolute path
    // that does not exist is a straightforward mistake, not something to
    // go looking for elsewhere.
    if (hintPath.is_relative()) {
        if (searchUpward(fs::current_path(ec), hintPath, tried, found)) return found;
        if (argv0 && *argv0) {
            const fs::path exeDir = fs::path(argv0).parent_path();
            if (!exeDir.empty() &&
                searchUpward(exeDir, hintPath, tried, found)) {
                return found;
            }
        }
    }

    std::string msg = "cannot find cns.bin.\nLooked in:\n";
    for (const auto& t : tried) msg += "  " + t + "\n";
    msg += "\nIf the data has not been built yet, run this from the project root:\n"
           "  python tools/fetch_cns.py\n"
           "  python tools/pack_cns.py\n"
           "Otherwise pass --data <directory>.";
    throw std::runtime_error(msg);
}

}  // namespace fly
