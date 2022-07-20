option(WITH_ONLINE_FOOTSTEP_PLANNER "Build with support for OnlineFootstepPlanner (Humanoid Navigation)" OFF)

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
  src/ExternalFootstepPlannerPlugin.cpp
  src/ExternalFootstepPlanner.cpp
  src/DummyPlanner.cpp
  src/InputConvertor.cpp
)

set(ExternalFootstepPlanner_HDR
  include/ExternalFootstepPlanner/ExternalFootstepPlannerPlugin.h
  include/ExternalFootstepPlanner/ExternalFootstepPlanner.h
  include/ExternalFootstepPlanner/DummyPlanner.h
  include/ExternalFootstepPlanner/InputConvertor.h
)

if(DEFINED CACHE{DEFAULT_EXTERNAL_PLANNER})
unset(DEFAULT_EXTERNAL_PLANNER CACHE)
endif()
if(WITH_ONLINE_FOOTSTEP_PLANNER)
  include(FindPkgConfig)
  pkg_check_modules(mc_plugin_footstep_plan_msgs REQUIRED IMPORTED_TARGET mc_plugin_footstep_plan_msgs)
  list(APPEND ExternalFootstepPlanner_SRC src/OnlineFootstepPlanner.cpp)
  list(APPEND ExternalFootstepPlanner_HDR include/ExternalFootstepPlanner/OnlineFootstepPlanner.h)
  set(DEFAULT_EXTERNAL_PLANNER "OnlineFootstepPlanner" CACHE STRING "Planner used by default by the plugin")
  set_property(CACHE DEFAULT_EXTERNAL_PLANNER PROPERTY STRINGS OnlineFootstepPlanner DummyPlanner)
  #set(DEFAULT_EXTERNAL_PLANNER "OnlineFootstepPlanner")
else()
  set(DEFAULT_EXTERNAL_PLANNER "DummyPlanner" CACHE STRING "Planner used by default by the plugin")
  set_property(CACHE DEFAULT_EXTERNAL_PLANNER PROPERTY STRINGS DummyPlanner)
  #set(DEFAULT_EXTERNAL_PLANNER "DummyPlanner")
endif()


option(AUTOLOAD_ExternalFootstepPlannerPlugin_PLUGIN "Automatically load ExternalFootstepPlanner plugin" OFF)
add_plugin(ExternalFootstepPlannerPlugin "${ExternalFootstepPlanner_SRC}" "${ExternalFootstepPlanner_HDR}")
target_include_directories(ExternalFootstepPlannerPlugin PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(ExternalFootstepPlannerPlugin PUBLIC mc_rtc::mc_rtc_ros ExternalFootstepPlanner_data)
set_target_properties(ExternalFootstepPlannerPlugin PROPERTIES OUTPUT_NAME ExternalFootstepPlanner)
if(WITH_ONLINE_FOOTSTEP_PLANNER)
  target_link_libraries(ExternalFootstepPlannerPlugin PUBLIC PkgConfig::mc_plugin_footstep_plan_msgs)
  target_compile_definitions(ExternalFootstepPlannerPlugin PRIVATE USE_ONLINE_FOOTSTEP_PLANNER)
endif()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/etc/ExternalFootstepPlannerPlugin.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/etc/ExternalFootstepPlannerPlugin.yaml)
install(FILES  DESTINATION ${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc/)
