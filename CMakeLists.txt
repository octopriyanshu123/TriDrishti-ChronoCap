cmake_minimum_required(VERSION 3.22)

project(chrono_cap LANGUAGES CXX)

set(I2W_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../i2w"
    CACHE PATH
    "Path to the i2w source tree")
set(CRAWLER_I2W_MSGS_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../crawler-i2w-msgs"
    CACHE PATH
    "Path to the crawler i2w message package")

if(NOT EXISTS "${I2W_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "i2w source not found at '${I2W_SOURCE_DIR}'")
endif()
if(NOT EXISTS "${CRAWLER_I2W_MSGS_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "crawler-i2w-msgs source not found at '${CRAWLER_I2W_MSGS_SOURCE_DIR}'")
endif()

add_subdirectory("${I2W_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/i2w")
add_subdirectory("${CRAWLER_I2W_MSGS_SOURCE_DIR}"
                 "${CMAKE_CURRENT_BINARY_DIR}/crawler_i2w_msgs")


# Lib 
add_library(chrono_cap SHARED
    src/core/record_writer.cpp
    src/core/record_reader.cpp
)

target_include_directories(chrono_cap PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test

)

target_link_libraries(chrono_cap PUBLIC pthread)

target_compile_options(chrono_cap PRIVATE -Wall -Wextra)

find_package(nlohmann_json REQUIRED)

# i2w logger Test 
# recorder
add_executable(i2wRecorder_node
  src/i2wRecorder.cpp
)
target_compile_features(i2wRecorder_node PRIVATE cxx_std_20)
target_include_directories(i2wRecorder_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(i2wRecorder_node PRIVATE
  i2w::i2w
  crawler_i2w_msgs::crawler_i2w_msgs
  chrono_cap
)
target_compile_definitions(i2wRecorder_node PRIVATE
    CONFIG_DIR="${CMAKE_SOURCE_DIR}/config")

install(TARGETS i2wRecorder_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})  

# replayer

add_executable(i2wReplayer_node
  src/i2wReplayer.cpp
)
target_compile_features(i2wReplayer_node PRIVATE cxx_std_20)
target_include_directories(i2wReplayer_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(i2wReplayer_node PRIVATE
  i2w::i2w
  crawler_i2w_msgs::crawler_i2w_msgs
  chrono_cap

)

install(TARGETS i2wReplayer_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})  