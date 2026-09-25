cmake_minimum_required(VERSION 3.14)
project(rmagine 
    LANGUAGES CXX C
    VERSION 2.4.3)# TODO update this version when merging into main-branch

set(RMAGINE_DEPENDENCY_MODE "Download" CACHE STRING "Choose the dependency management mode.")
set_property(CACHE RMAGINE_DEPENDENCY_MODE PROPERTY STRINGS "Local" "Auto" "Download")
# Local: Use local packages only. This is the old default behavior until v2.3.0
# Auto: Use local package if available and download otherwise.
# Download: Always download (this repository's default for reproducible builds).

# To still use FetchContent_Populate for non CMake projects 
# without triggering a warning
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()

include(GNUInstallDirs)

set(rmagine_DEFINITIONS "-DRMAGINE_VERSION=\"${rmagine_VERSION}\"")
list(APPEND rmagine_DEFINITIONS "-DRMAGINE_VERSION_MAJOR=${rmagine_VERSION_MAJOR}")
list(APPEND rmagine_DEFINITIONS "-DRMAGINE_VERSION_MINOR=${rmagine_VERSION_MINOR}")
list(APPEND rmagine_DEFINITIONS "-DRMAGINE_VERSION_PATCH=${rmagine_VERSION_PATCH}")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()


set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_TMP_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tmp)

list(APPEND CMAKE_MODULE_PATH ${rmagine_SOURCE_DIR}/cmake)

# printing in color
if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

include(FetchContent)

################################
## REQUIRED DEPS

find_package(Boost CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

####################
## ASSIMP: MeshIO ##
####################
find_package(assimp REQUIRED)

#######################
## TBB: Threading ##
#######################
find_package(TBB REQUIRED)

######################
## Build Components ##
######################

add_definitions(${rmagine_DEFINITIONS})

set(RMAGINE_LIBRARIES)
set(RMAGINE_STATIC_LIBRARIES)

### RMAGINE CORE COMPONENT
add_subdirectory(src/rmagine_core)

### RMAGINE EMBREE COMPONENT
add_subdirectory(src/rmagine_embree)

### ALL LIBS
message(STATUS "${BoldCyan}Components being built:${ColourReset}")
foreach(LIBRARY ${RMAGINE_LIBRARIES})
    message(STATUS "- ${BoldGreen}${LIBRARY}${ColourReset}")
endforeach()

# CMAKE FIND SCRIPT
include(CMakePackageConfigHelpers)

configure_package_config_file(cmake/rmagine-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/rmagine-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rmagine-${rmagine_VERSION}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/rmagine-config-version.cmake
    COMPATIBILITY SameMajorVersion
)

## INSTALL
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rmagine-config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/rmagine-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rmagine-${rmagine_VERSION}
    COMPONENT core
)

install(FILES package.xml
    DESTINATION share/rmagine-${rmagine_VERSION}
    COMPONENT core
)
