# Copyright (c) 2018-2019, CNRS-UM LIRMM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

set(PROJECT_NAME lipm_walking_controller)
set(PROJECT_DESCRIPTION "Walking controller based on linear inverted pendulum tracking")
set(PROJECT_URL https://github.com/stephane-caron/lipm_walking_controller/)
set(PROJECT_VERSION 1.6.0)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(INSTALL_DOCUMENTATION OFF CACHE BOOL "")
set(INSTALL_GENERATED_HEADERS OFF CACHE BOOL "" FORCE)
set(INSTALL_PKG_CONFIG_FILE OFF CACHE BOOL "" FORCE)
set(CXX_DISABLE_WERROR ON)

include(cmake/base.cmake)

project(${PROJECT_NAME} CXX)

add_project_dependency(copra REQUIRED)
add_project_dependency(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc must be built with ROS support for this controller to work")
endif()
add_project_dependency(eigen-quadprog REQUIRED)

if(EXISTS "${MC_RTC_LIBDIR}/mc_controller/etc/LIPMWalking.conf")
  message(FATAL_ERROR "\n \
############################################################ \n \
   Please uninstall the previous version (make uninstall)    \n \
############################################################ \n \
")
endif()

add_subdirectory(plugins/ExternalFootstepPlanner)
add_subdirectory(src)

set(PACKAGE_EXTRA_MACROS "set(LIPM_WALKING_STATES_LIBRARIES \"${MC_CONTROLLER_INSTALL_PREFIX}/lipm_walking_controller/states\")
set(LIPM_WALKING_STATES_FILES \"\${LIPM_WALKING_STATES_LIBRARIES}/data\")")
