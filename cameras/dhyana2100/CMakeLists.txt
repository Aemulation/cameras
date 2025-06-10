cmake_minimum_required(VERSION 3.12)
project(
  dhyana2100
  VERSION 1.0.0
  LANGUAGES CXX)

set(DHYANA_SDK_PATH ${PROJECT_SOURCE_DIR}/dhyana-sdk)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt
  GIT_TAG e69e5f977d458f2650bb346dadf2ad30c5320281) # 10.2.1
FetchContent_MakeAvailable(fmt)

add_library(${PROJECT_NAME} SHARED)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-O2 -Wall -Wextra -Wpedantic)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options(/O2 /W4 /permissive-)
endif()

file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS src/*.cpp src/*.cu)
target_sources(${PROJECT_NAME} PRIVATE ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME}
                           PUBLIC ${PROJECT_SOURCE_DIR}/include/)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)

add_compile_definitions(${PROJECT_NAME} PUBLIC TUCAM_TARGETOS_IS_WINDOWS)
add_compile_definitions(${PROJECT_NAME} PUBLIC DHYANA_EXPORTS)
add_compile_definitions(${PROJECT_NAME} PUBLIC NOMINMAX)

if(NOT (WIN32))
    set(CMAKE_C_COMPILER "/usr/bin/x86_64-w64-mingw32-cc")
    set(CMAKE_CXX_COMPILER "/usr/bin/x86_64-w64-mingw32-c++")
endif()

target_include_directories(${PROJECT_NAME} SYSTEM
                           PUBLIC ${DHYANA_SDK_PATH}/inc/)

target_link_libraries(
  ${PROJECT_NAME} PUBLIC ${DHYANA_SDK_PATH}/lib/x64/TUCam.lib
                         fmt::fmt-header-only)
