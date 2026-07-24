cmake_minimum_required(VERSION 3.20)
project(ConstellationSim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
  add_compile_options(/W4 /O2)
else()
  add_compile_options(-Wall -Wextra -O3 -march=native)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-fopenmp-simd)
  endif()
endif()

# --------------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------------
include(FetchContent)

FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
)
FetchContent_MakeAvailable(json)

find_package(Threads REQUIRED)

# --------------------------------------------------------------------------
# Source groups
# --------------------------------------------------------------------------
set(CORE_SOURCES
  src/core/ConfigLoader.cpp
  src/core/SimulationEngine.cpp
)

set(ORBIT_SOURCES
  src/orbit/OrbitalElements.cpp
  src/orbit/Satellite.cpp
  src/orbit/Propagator.cpp
)

set(ENVIRONMENT_SOURCES
  src/environment/SunModel.cpp
  src/environment/MoonModel.cpp
  src/environment/AtmosphereModel.cpp
  src/environment/EclipseModel.cpp
)

set(PHYSICS_SOURCES
  src/physics/Gravity.cpp
  src/physics/J2Perturbation.cpp
  src/physics/ZonalHarmonics.cpp
  src/physics/AtmosphericDrag.cpp
  src/physics/SolarRadiationPressure.cpp
  src/physics/ThirdBodyGravity.cpp
)

set(CONSTELLATION_SOURCES
  src/constellation/Plane.cpp
  src/constellation/Constellation.cpp
  src/constellation/Deployment.cpp
)

set(METRICS_SOURCES
  src/metrics/MetricsCollector.cpp
)

set(MONTECARLO_SOURCES
  src/montecarlo/ParameterSweep.cpp
  src/montecarlo/SimulationBatch.cpp
  src/montecarlo/ExperimentManager.cpp
)

set(OUTPUT_SOURCES
  src/output/CsvWriter.cpp
  src/output/OutputManager.cpp
)

# --------------------------------------------------------------------------
# Core simulation library (headless, no GL dependency)
# --------------------------------------------------------------------------
add_library(constellation_core STATIC
  ${CORE_SOURCES}
  ${ORBIT_SOURCES}
  ${ENVIRONMENT_SOURCES}
  ${PHYSICS_SOURCES}
  ${CONSTELLATION_SOURCES}
  ${METRICS_SOURCES}
  ${MONTECARLO_SOURCES}
  ${OUTPUT_SOURCES}
)

target_include_directories(constellation_core PUBLIC src)
target_link_libraries(constellation_core PUBLIC
  nlohmann_json::nlohmann_json
  Threads::Threads
)

# --------------------------------------------------------------------------
# Main executable
# --------------------------------------------------------------------------
add_executable(ConstellationSim src/main.cpp)
target_link_libraries(ConstellationSim PRIVATE constellation_core)

# --------------------------------------------------------------------------
# Optional visualization (requires VGL)
# --------------------------------------------------------------------------
option(BUILD_VIZ "Build with VGL visualization support" OFF)

if(BUILD_VIZ)
  set(VGL_LOCAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../VGL")
  if(EXISTS "${VGL_LOCAL_PATH}/CMakeLists.txt")
    add_subdirectory(${VGL_LOCAL_PATH} vgl_build)
  else()
    FetchContent_Declare(
      vgl
      GIT_REPOSITORY https://github.com/vcampos4545/VGL.git
      GIT_TAG main
    )
    FetchContent_MakeAvailable(vgl)
  endif()

  # Dear ImGui — GLFW + OpenGL3 backend
  FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.6
  )
  FetchContent_MakeAvailable(imgui)

  add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
  )
  target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
  )
  # Tell imgui_impl_opengl3 to use the GLEW loader (VGL already calls glewInit()).
  target_compile_definitions(imgui_lib PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLEW)
  target_link_libraries(imgui_lib PUBLIC vgl)

  target_sources(constellation_core PRIVATE
    src/viz/Renderer.cpp
    src/viz/SatelliteRenderer.cpp
  )
  target_link_libraries(constellation_core PUBLIC vgl imgui_lib)
  target_compile_definitions(constellation_core PUBLIC CONSTELLATION_VIZ_ENABLED)
endif()

# --------------------------------------------------------------------------
# Copy config directories and resources to build tree for convenience.
# Uses add_custom_target(ALL) so the copy runs on every build, not only
# when C++ sources change (POST_BUILD would skip it for JSON-only edits).
# --------------------------------------------------------------------------
add_custom_target(copy_assets ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/config"
    "$<TARGET_FILE_DIR:ConstellationSim>/config"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/montecarlo"
    "$<TARGET_FILE_DIR:ConstellationSim>/montecarlo"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/resources"
    "$<TARGET_FILE_DIR:ConstellationSim>/resources"
  COMMENT "Syncing config files and resources to build directory"
  DEPENDS ConstellationSim
)
