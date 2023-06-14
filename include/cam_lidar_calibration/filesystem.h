#ifndef src_filesystem_h
#define src_filesystem_h

#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#define FILESYSTEM_SUPPORTED
#elif defined(__cpp_lib_filesystem)
#include <filesystem>
namespace fs = std::filesystem;
#define FILESYSTEM_SUPPORTED
#elif defined(__cpp_lib_experimental_filesystem)
#include <experimental/filesystem>
using fs = std::experimental::filesystem;
#define EXPERIMENTAL_FILESYSTEM_SUPPORTED
#elif __has_include(<boost/filesystem.hpp>)
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#define BOOST_FILESYSTEM_SUPPORTED
#else
#error "No filesystem support"
#endif

namespace cam_lidar_calibration
{

    inline std::size_t number_of_files_in_directory(const fs::path& path)
    {
        return (std::size_t) std::distance(fs::directory_iterator{path},
                                           fs::directory_iterator{});
    }

}

#endif //src_filesystem_h
