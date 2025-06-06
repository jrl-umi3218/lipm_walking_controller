option(ONLINE_FOOTSTEP_PLANNER
       "Build with support for OnlineFootstepPlanner (Humanoid Navigation)" OFF)
option(HYBRID_PLANNER "Build with support for the Hybrid Planner (ISMPC)" ON)

set(ExternalFootstepPlanner_data_SRC src/Plan.cpp src/Request.cpp)

set(ExternalFootstepPlanner_data_HDR include/ExternalFootstepPlanner/Plan.h
                                     include/ExternalFootstepPlanner/Request.h)
add_library(
  ExternalFootstepPlanner_data SHARED "${ExternalFootstepPlanner_data_SRC}"
                                      "${ExternalFootstepPlanner_data_HDR}")
if(TARGET Eigen3::Eigen)
  target_link_libraries(ExternalFootstepPlanner_data PUBLIC Eigen3::Eigen)
else()
  target_include_directories(ExternalFootstepPlanner_data SYSTEM
                             INTERFACE "${EIGEN3_INCLUDE_DIR}")
endif()
target_include_directories(
  ExternalFootstepPlanner_data
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(
  TARGETS ExternalFootstepPlanner_data
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/ExternalFootstepPlanner DESTINATION include)

find_package(ament_cmake QUIET)
if(NOT TARGET mc_rtc::mc_rtc_ros OR ${ament_cmake_FOUND})
  message(
    WARNING
      "mc_rtc was built without ROS support, the ExternalFootstepPlanner plugin will not be built"
  )
  return()
endif()

set(ExternalFootstepPlanner_SRC src/ExternalFootstepPlanner.cpp
                                src/DummyPlanner.cpp)

set(ExternalFootstepPlanner_HDR
    include/ExternalFootstepPlanner/ExternalFootstepPlanner.h
    include/ExternalFootstepPlanner/DummyPlanner.h)

add_library(ExternalFootstepPlanner SHARED ${ExternalFootstepPlanner_SRC}
                                           ${ExternalFootstepPlanner_HDR})
target_link_libraries(ExternalFootstepPlanner
                      PUBLIC ExternalFootstepPlanner_data mc_rtc::mc_control)
install(
  TARGETS ExternalFootstepPlanner
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

set(ExternalFootstepPlannerPlugin_HDR
    include/ExternalFootstepPlanner/ExternalFootstepPlannerPlugin.h)

set(ExternalFootstepPlannerPlugin_SRC src/ExternalFootstepPlannerPlugin.cpp)

option(AUTOLOAD_ExternalFootstepPlannerPlugin_PLUGIN
       "Automatically load ExternalFootstepPlanner plugin" OFF)
add_plugin(ExternalFootstepPlannerPlugin "${ExternalFootstepPlannerPlugin_SRC}"
           "${ExternalFootstepPlannerPlugin_HDR}")
target_include_directories(
  ExternalFootstepPlannerPlugin
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(ExternalFootstepPlannerPlugin
                      PUBLIC ExternalFootstepPlanner)
target_link_libraries(ExternalFootstepPlannerPlugin PUBLIC mc_rtc::mc_rtc_ros)
set_target_properties(ExternalFootstepPlannerPlugin
                      PROPERTIES OUTPUT_NAME ExternalFootstepPlannerPlugin)

if(ONLINE_FOOTSTEP_PLANNER)
  include(FindPkgConfig)
  pkg_check_modules(mc_plugin_footstep_plan_msgs REQUIRED IMPORTED_TARGET
                    mc_plugin_footstep_plan_msgs)
  message(STATUS "Building with OnlineFootstepPlanner support")
  add_library(
    OnlineFootstepPlanner SHARED
    src/OnlineFootstepPlanner.cpp
    include/ExternalFootstepPlanner/OnlineFootstepPlanner.h
    include/ExternalFootstepPlanner/InputConvertor.h src/InputConvertor.cpp)
  target_link_libraries(
    OnlineFootstepPlanner PUBLIC mc_rtc::mc_rtc_ros
                                 PkgConfig::mc_plugin_footstep_plan_msgs)
  target_link_libraries(OnlineFootstepPlanner PUBLIC ExternalFootstepPlanner)
  install(
    TARGETS OnlineFootstepPlanner
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

  set(DEFAULT_EXTERNAL_PLANNER "OnlineFootstepPlanner")
  target_link_libraries(ExternalFootstepPlannerPlugin
                        PUBLIC OnlineFootstepPlanner)
  target_compile_definitions(ExternalFootstepPlannerPlugin
                             PRIVATE USE_ONLINE_FOOTSTEP_PLANNER)
else()
  set(DEFAULT_EXTERNAL_PLANNER "DummyPlanner")
endif()

if(HYBRID_PLANNER)
  add_library(
    HybridPlanner SHARED src/HybridPlanner.cpp
                         include/ExternalFootstepPlanner/HybridPlanner.h)
  target_link_libraries(HybridPlanner PUBLIC ExternalFootstepPlanner)
  target_link_libraries(ExternalFootstepPlannerPlugin PUBLIC HybridPlanner)
  target_compile_definitions(ExternalFootstepPlannerPlugin
                             PRIVATE USE_HYBRID_PLANNER)
  install(
    TARGETS HybridPlanner
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/etc/ExternalFootstepPlannerPlugin.in.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/etc/ExternalFootstepPlannerPlugin.yaml)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/ExternalFootstepPlannerPlugin.yaml
        DESTINATION ${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc/)
