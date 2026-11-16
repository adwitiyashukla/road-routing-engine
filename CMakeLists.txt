cmake_minimum_required(VERSION 3.16)
project(road-routing-engine CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "build type" FORCE)
endif()

option(RR_SANITIZE_ADDRESS "build with the address and undefined sanitizers" OFF)
option(RR_SANITIZE_THREAD "build with the thread sanitizer" OFF)
option(RR_WERROR "turn warnings into errors" OFF)

if(MSVC)
  add_compile_options(/W4 /permissive- /Zc:preprocessor /utf-8 /EHsc /bigobj)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
  if(RR_WERROR)
    add_compile_options(/WX)
  endif()
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(RR_WERROR)
    add_compile_options(-Werror)
  endif()
endif()

if(RR_SANITIZE_ADDRESS AND RR_SANITIZE_THREAD)
  message(FATAL_ERROR "pick either RR_SANITIZE_ADDRESS or RR_SANITIZE_THREAD")
endif()

if(RR_SANITIZE_ADDRESS)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=address,undefined)
endif()

if(RR_SANITIZE_THREAD)
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
  add_link_options(-fsanitize=thread)
endif()

find_package(Threads REQUIRED)

add_library(roadroute_core STATIC
  src/ch.cpp
  src/contract.cpp
  src/dimacs.cpp
  src/generator.cpp
  src/geo.cpp
  src/graph.cpp
  src/gzip.cpp
  src/http.cpp
  src/io.cpp
  src/search.cpp
  src/service.cpp
  src/thread_pool.cpp
)
target_include_directories(roadroute_core PUBLIC src)
target_link_libraries(roadroute_core PUBLIC Threads::Threads)
if(WIN32)
  target_link_libraries(roadroute_core PUBLIC ws2_32)
endif()

add_executable(roadroute src/main.cpp)
target_link_libraries(roadroute PRIVATE roadroute_core)

enable_testing()
foreach(test io_test geo_test graph_test search_test ch_test http_test)
  add_executable(${test} tests/${test}.cpp)
  target_link_libraries(${test} PRIVATE roadroute_core)
  target_include_directories(${test} PRIVATE tests)
  add_test(NAME ${test} COMMAND ${test})
endforeach()
