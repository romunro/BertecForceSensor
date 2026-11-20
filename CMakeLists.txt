cmake_minimum_required(VERSION 3.14)

project(
  BertecForceSensor
  LANGUAGES CXX
  VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_MC_RTC_PLUGIN "Build the mc_rtc GlobalPlugin (BertecForceSensor)" ON)
option(BUILD_CLI "Build the standalone C++ CLI test binary" ON)

find_package(Threads REQUIRED)

add_library(bertec_am6817 SHARED src/BertecAM6817.cpp)
target_include_directories(
  bertec_am6817
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_link_libraries(bertec_am6817 PUBLIC Threads::Threads)

install(TARGETS bertec_am6817 EXPORT BertecForceSensorTargets LIBRARY DESTINATION lib)
install(DIRECTORY include/bertec_am6817 DESTINATION include)

if(BUILD_CLI)
  add_executable(bertec_am6817_cli src/cli_main.cpp)
  target_link_libraries(bertec_am6817_cli PRIVATE bertec_am6817)
  install(TARGETS bertec_am6817_cli RUNTIME DESTINATION bin)
endif()

if(BUILD_MC_RTC_PLUGIN)
  if(NOT TARGET mc_rtc::mc_control)
    find_package(mc_rtc REQUIRED)
  endif()

  configure_file(etc/BertecForceSensor.in.yaml
                 "${CMAKE_CURRENT_BINARY_DIR}/etc/BertecForceSensor.yaml")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/BertecForceSensor.yaml"
          DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

  set(plugin_SRC src/BertecForceSensorPlugin.cpp)
  set(plugin_HDR src/BertecForceSensorPlugin.h)

  add_plugin(BertecForceSensor "${plugin_SRC}" "${plugin_HDR}")
  target_link_libraries(BertecForceSensor PUBLIC bertec_am6817 mc_rtc::mc_control)
endif()
