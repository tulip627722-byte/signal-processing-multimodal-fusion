cmake_minimum_required(VERSION 3.15)
project(LivoxBridge LANGUAGES CXX)

if(NOT DEFINED LIVOX_SDK_ROOT)
  message(FATAL_ERROR "Set LIVOX_SDK_ROOT to the official Livox-SDK checkout")
endif()

add_subdirectory("${LIVOX_SDK_ROOT}" "${CMAKE_BINARY_DIR}/livox-sdk" EXCLUDE_FROM_ALL)

add_library(LivoxBridge SHARED LivoxBridge.cpp LivoxBridge.h)
target_compile_features(LivoxBridge PRIVATE cxx_std_14)
target_compile_definitions(LivoxBridge PRIVATE LIVOX_BRIDGE_EXPORTS WIN32_LEAN_AND_MEAN)
target_include_directories(LivoxBridge PRIVATE "${LIVOX_SDK_ROOT}/sdk_core/include")
target_link_libraries(LivoxBridge PRIVATE livox_sdk_static ws2_32)

set_target_properties(LivoxBridge PROPERTIES
  OUTPUT_NAME "LivoxBridge"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Release")
