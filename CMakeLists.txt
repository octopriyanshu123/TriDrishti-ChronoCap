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
    src/record_writer.cpp
    src/record_reader.cpp
)

target_include_directories(chrono_cap PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(chrono_cap PUBLIC pthread)

target_compile_options(chrono_cap PRIVATE -Wall -Wextra)

find_package(nlohmann_json REQUIRED)
                 

add_executable(sub_node
  test/sub.cpp
)
target_compile_features(sub_node PRIVATE cxx_std_20)
target_include_directories(sub_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(sub_node PRIVATE
  i2w::i2w
  crawler_i2w_msgs::crawler_i2w_msgs
)

install(TARGETS sub_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

add_executable(pub_node
  test/pub.cpp
)
target_compile_features(pub_node PRIVATE cxx_std_20)
target_include_directories(pub_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(pub_node PRIVATE
  i2w::i2w
  crawler_i2w_msgs::crawler_i2w_msgs
)

install(TARGETS pub_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})


# Logger sub 
add_executable(logger_sub_node
  test/logger_sub.cpp
)
target_compile_features(logger_sub_node PRIVATE cxx_std_20)
target_include_directories(logger_sub_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(logger_sub_node PRIVATE
  i2w::i2w
  crawler_i2w_msgs::crawler_i2w_msgs
)

install(TARGETS logger_sub_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# recorder
add_executable(recorder_node
  test/recorder.cpp
)
target_compile_features(recorder_node PRIVATE cxx_std_20)
target_include_directories(recorder_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(recorder_node PRIVATE
  i2w::i2w
  crawler_i2w_msgs::crawler_i2w_msgs
)

install(TARGETS recorder_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})  

# replayer

add_executable(replayer_node
  test/replayer.cpp
)
target_compile_features(replayer_node PRIVATE cxx_std_20)
target_include_directories(replayer_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(replayer_node PRIVATE
  i2w::i2w
  crawler_i2w_msgs::crawler_i2w_msgs
)

install(TARGETS replayer_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})  

  # Test 

add_executable(write_test
  test/write.cpp
)

add_executable(read_test
  test/mmapRead.cpp
)



add_executable(replay_log_test
  test/replay_log.cpp
)

add_executable(robot_logger
    src/log.cpp
    src/core/logger.cpp
)

target_include_directories(robot_logger
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)



add_executable(robot_logger_replay
    src/replayLog.cpp

    src/core/logger.cpp
)

target_include_directories(robot_logger_replay
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)


