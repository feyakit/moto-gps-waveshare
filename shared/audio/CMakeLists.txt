cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(moto_audio LANGUAGES CXX)
endif()

if(NOT TARGET moto_nav_core)
  add_subdirectory(
    "${CMAKE_CURRENT_LIST_DIR}/../nav_core"
    "${CMAKE_CURRENT_BINARY_DIR}/nav_core"
  )
endif()

add_library(moto_audio STATIC
  src/chime_policy.cpp
)

target_include_directories(moto_audio
  PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/include"
)

# ManeuverType comes from the provider-neutral route vocabulary in port_api,
# which nav_core already exports; the policy must speak the same enum the
# navigation snapshot does.
target_link_libraries(moto_audio PUBLIC moto_nav_core)
target_compile_features(moto_audio PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(moto_audio PRIVATE /W4)
else()
  target_compile_options(moto_audio PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -fno-exceptions
    -fno-rtti
  )
endif()
