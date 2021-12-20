// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: http://www.viva64.com
#include "FileIO.h"

#if _WIN32

#include <filesystem>
#if _MSC_VER >= 1920
namespace fs = std::filesystem;
#elif _MSC_VER >= 1910
namespace fs = std::experimental::filesystem;
#else
#error please use vs2017 or greater
#endif

#else

#if __cplusplus >= 201703L
#include <filesystem>
namespace fs = std::filesystem;
#elif __cplusplus >= 201103L
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#error please update your gcc version
#endif

#endif

bool MakeDirectories(const std::string directories) {
    return fs::create_directories(directories.c_str());
}