option(ONLINE_FOOTSTEP_PLANNER "Build with support for OnlineFootstepPlanner (Humanoid Navigation)" OFF)

set(ExternalFootstepPlanner_data_SRC
  src/Plan.cpp
  src/Request.cpp
)

set(ExternalFootstepPlanner_data_HDR
  include/ExternalFootstepPlanner/Plan.h
  include/ExternalFootstepPlanner/Request.h
)
add_library(ExternalFootstepPlanner_data SHARED "${ExternalFootstepPlanner_data_SRC}" "${ExternaleFootstepPlanner_data_HDR}")
if(TARGET Eigen3::Eigen)
  target_link_libraries(ExternalFootstepPlanner_data PUBLIC Eigen3::Eigen)
else()
  target_include_directories(ExternalFootstepPlanner_data SYSTEM INTERFACE "${EIGEN3_INCLUDE_DIR}")
endif()
target_include_directories(ExternalFootstepPlanner_data PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(TARGETS ExternalFootstepPlanner_data
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
  DIRECTORY include/ExternalFootstepPlanner DESTINATION include)

set(ExternalFootstepPlanner_SRC
  src/ExternalFootstepPlanner.cpp
  src/DummyPlanner.cpp
)

set(ExternalFootstepPlanner_HDR
  include/ExternalFootstepPlanner/ExternalFootstepPlanner.h
  include/ExternalFootstepPlanner/DummyPlanner.h
)

add_library(ExternalFootstepPlanner SHARED ${ExternalFootstepPlanner_SRC} ${ExternalFootstepPlanner_HDR})
target_link_libraries(ExternalFootstepPlanner PUBLIC ExternalFootstepPlanner_data mc_rtc::mc_control)
install(TARGETS ExternalFootstepPlanner
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

set(ExternalFootstepPlannerPlugin_HDR
  include/ExternalFootstepPlanner/ExternalFootstepPlannerPlugin.h
)

set(ExternalFootstepPlannerPlugin_SRC
  src/ExternalFootstepPlannerPlugin.cpp
)

option(AUTOLOAD_ExternalFootstepPlannerPlugin_PLUGIN "Automatically load ExternalFootstepPlanner plugin" OFF)
add_plugin(ExternalFootstepPlannerPlugin "${ExternalFootstepPlannerPlugin_SRC}" "${ExternalFootstepPlannerPlugin_HDR}")
target_include_directories(ExternalFootstepPlannerPlugin PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(ExternalFootstepPlannerPlugin PUBLIC ExternalFootstepPlanner)
set_target_properties(ExternalFootstepPlannerPlugin PROPERTIES OUTPUT_NAME ExternalFootstepPlannerPlugin)

if(ONLINE_FOOTSTEP_PLANNER)
  include(FindPkgConfig)
  pkg_check_modules(mc_plugin_footstep_plan_msgs REQUIRED IMPORTED_TARGET mc_plugin_footstep_plan_msgs)
  message(STATUS "Building with OnlineFootstepPlanner support")
  add_library(OnlineFootstepPlanner SHARED src/OnlineFootstepPlanner.cpp include/ExternalFootstepPlanner/OnlineFootstepPlanner.h include/ExternalFootstepPlanner/InputConvertor.h src/InputConvertor.cpp)
  target_link_libraries(OnlineFootstepPlanner PUBLIC mc_rtc::mc_rtc_ros PkgConfig::mc_plugin_footstep_plan_msgs)
  target_link_libraries(OnlineFootstepPlanner PUBLIC ExternalFootstepPlanner)
  install(TARGETS OnlineFootstepPlanner
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

  set(DEFAULT_EXTERNAL_PLANNER "OnlineFootstepPlanner")
  target_link_libraries(ExternalFootstepPlannerPlugin PUBLIC OnlineFootstepPlanner)
  target_compile_definitions(ExternalFootstepPlannerPlugin PRIVATE USE_ONLINE_FOOTSTEP_PLANNER)
else()
  set(DEFAULT_EXTERNAL_PLANNER "DummyPlanner")
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/etc/ExternalFootstepPlannerPlugin.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/etc/ExternalFootstepPlannerPlugin.yaml)
install(FILES  DESTINATION ${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc/)
