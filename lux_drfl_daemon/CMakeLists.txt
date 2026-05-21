cmake_minimum_required(VERSION 3.16)
project(lux_drfl_daemon CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Options ───────────────────────────────────────────────────────────────────
set(DRCF_VERSION "3" CACHE STRING "DRCF protocol version (2 or 3)")

# ── Arch detection ────────────────────────────────────────────────────────────
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(DRFL_ARCH "arm64")
else()
    set(DRFL_ARCH "amd64")
endif()

# ── Ubuntu version detection ──────────────────────────────────────────────────
set(UBUNTU_VERSION "" CACHE STRING "Ubuntu version override (e.g. 18.04, 22.04)")
if(NOT UBUNTU_VERSION)
    execute_process(
        COMMAND lsb_release -rs
        OUTPUT_VARIABLE UBUNTU_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
endif()
if(NOT UBUNTU_VERSION)
    # Fallback: pick whichever version directory exists
    file(GLOB _candidates
        LIST_DIRECTORIES true
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/API-DRFL/library/Linux/64bits/${DRFL_ARCH}/*")
    list(GET _candidates 0 _first)
    if(_first)
        get_filename_component(UBUNTU_VERSION "${_first}" NAME)
    else()
        set(UBUNTU_VERSION "22.04")
    endif()
endif()

# ── API-DRFL paths ────────────────────────────────────────────────────────────
# Actual repo layout: library/Linux/64bits/{amd64|arm64}/{ubuntu_ver}/
set(DRFL_ROOT    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/API-DRFL")
set(DRFL_INCLUDE "${DRFL_ROOT}/include")
set(DRFL_LIB_DIR "${DRFL_ROOT}/library/Linux/64bits/${DRFL_ARCH}/${UBUNTU_VERSION}")
set(DRFL_LIB     "${DRFL_LIB_DIR}/libDRFL.a")

if(NOT EXISTS "${DRFL_INCLUDE}/DRFLEx.h")
    message(FATAL_ERROR
        "DRFLEx.h not found at ${DRFL_INCLUDE}.\n"
        "Run: git submodule update --init third_party/API-DRFL")
endif()

if(NOT EXISTS "${DRFL_LIB}")
    # Try other Ubuntu versions under same arch
    file(GLOB_RECURSE DRFL_LIB_CANDIDATES
        "${DRFL_ROOT}/library/Linux/64bits/${DRFL_ARCH}/*/libDRFL.a")
    list(GET DRFL_LIB_CANDIDATES 0 DRFL_LIB)
    if(NOT DRFL_LIB)
        message(FATAL_ERROR
            "libDRFL.a not found under ${DRFL_ROOT}/library/Linux/64bits/${DRFL_ARCH}/\n"
            "Tried Ubuntu version: ${UBUNTU_VERSION}")
    endif()
    get_filename_component(DRFL_LIB_DIR "${DRFL_LIB}" DIRECTORY)
endif()

# Doosan bundles Poco shared libs alongside libDRFL.a — use them directly.
# No system libpoco-dev needed.
set(POCO_FOUNDATION_SO "${DRFL_LIB_DIR}/libPocoFoundation.so")
set(POCO_NET_SO        "${DRFL_LIB_DIR}/libPocoNet.so")

if(NOT EXISTS "${POCO_FOUNDATION_SO}")
    message(FATAL_ERROR "Bundled libPocoFoundation.so not found at ${DRFL_LIB_DIR}")
endif()

message(STATUS "DRFL arch:         ${DRFL_ARCH}")
message(STATUS "DRFL Ubuntu ver:   ${UBUNTU_VERSION}")
message(STATUS "DRFL lib dir:      ${DRFL_LIB_DIR}")
message(STATUS "DRCF version:      ${DRCF_VERSION}")

# ── nlohmann/json ─────────────────────────────────────────────────────────────
find_path(NLOHMANN_INCLUDE nlohmann/json.hpp
    PATHS "${CMAKE_CURRENT_SOURCE_DIR}/third_party"
    NO_DEFAULT_PATH)

if(NOT NLOHMANN_INCLUDE)
    include(FetchContent)
    FetchContent_Declare(
        nlohmann_json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp
        DOWNLOAD_NO_EXTRACT TRUE
        DOWNLOAD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann"
    )
    FetchContent_MakeAvailable(nlohmann_json)
    set(NLOHMANN_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
endif()

# ── libcurl ───────────────────────────────────────────────────────────────────
find_package(CURL REQUIRED)

# ── Target ────────────────────────────────────────────────────────────────────
add_executable(drfl_daemon src/drfl_daemon.cpp)

# Runtime lib path: Poco .so files are copied to /app/lib in the Docker image.
# For local dev runs, also search the source lib dir.
set(RPATH_ENTRIES "/app/lib" "${DRFL_LIB_DIR}")
string(JOIN ":" RPATH_STRING ${RPATH_ENTRIES})

target_include_directories(drfl_daemon PRIVATE
    ${DRFL_INCLUDE}
    ${NLOHMANN_INCLUDE}
)

target_compile_definitions(drfl_daemon PRIVATE
    DRCF_VERSION=${DRCF_VERSION}
)

target_link_libraries(drfl_daemon PRIVATE
    ${DRFL_LIB}
    ${POCO_FOUNDATION_SO}
    ${POCO_NET_SO}
    CURL::libcurl
    pthread
    dl
)

target_link_options(drfl_daemon PRIVATE
    "-Wl,-rpath,${RPATH_STRING}"
)

target_compile_options(drfl_daemon PRIVATE
    -Wall -Wextra -Wno-unused-parameter
)

# ── Install helper: expose lib dir for Dockerfile COPY ───────────────────────
# Creates build/poco_libs/ with the .so files needed at runtime
file(GLOB POCO_SOS "${DRFL_LIB_DIR}/libPoco*.so*")
foreach(_so ${POCO_SOS})
    get_filename_component(_so_name "${_so}" NAME)
    configure_file("${_so}" "${CMAKE_BINARY_DIR}/poco_libs/${_so_name}" COPYONLY)
endforeach()
