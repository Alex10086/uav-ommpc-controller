include(FetchContent)
include(GNUInstallDirs)

# OSQP 0.6.3 and its pinned QDLDL submodule, matching the reference controller API.
# Build inside _deps; do not install or depend on a system-wide solver.
set(UNITTESTS OFF CACHE BOOL "Build OSQP tests" FORCE)
set(ENABLE_MKL_PARDISO OFF CACHE BOOL "Use MKL in OSQP" FORCE)
set(DFLOAT OFF CACHE BOOL "Use single-precision OSQP" FORCE)
set(DLONG ON CACHE BOOL "Use 64-bit OSQP indices" FORCE)
set(PRINTING OFF CACHE BOOL "Enable OSQP printing" FORCE)
set(PROFILING ON CACHE BOOL "Enable OSQP timing" FORCE)
set(CTRLC OFF CACHE BOOL "Install OSQP signal handler" FORCE)
FetchContent_Declare(osqp
  GIT_REPOSITORY https://github.com/osqp/osqp.git
  GIT_TAG 0dd00a578cf1c2691c5c379965d504c75bf6cfad
  GIT_SUBMODULES lin_sys/direct/qdldl/qdldl_sources)
FetchContent_GetProperties(osqp)
if(NOT osqp_POPULATED)
  FetchContent_Populate(osqp)
  add_subdirectory(${osqp_SOURCE_DIR} ${osqp_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
