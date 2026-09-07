cmake_minimum_required(VERSION 3.22)

project(stm LANGUAGES CXX)

# set(I2W_SOURCE_DIR
#     "${CMAKE_CURRENT_SOURCE_DIR}/../i2w"
#     CACHE PATH
#     "Path to the i2w source tree")
# set(CRAWLER_I2W_MSGS_SOURCE_DIR
#     "${CMAKE_CURRENT_SOURCE_DIR}/../crawler-i2w-msgs"
#     CACHE PATH
#     "Path to the crawler i2w message package")

# if(NOT EXISTS "${I2W_SOURCE_DIR}/CMakeLists.txt")
#   message(FATAL_ERROR "i2w source not found at '${I2W_SOURCE_DIR}'")
# endif()
# if(NOT EXISTS "${CRAWLER_I2W_MSGS_SOURCE_DIR}/CMakeLists.txt")
#   message(FATAL_ERROR
#     "crawler-i2w-msgs source not found at '${CRAWLER_I2W_MSGS_SOURCE_DIR}'")
# endif()

# add_subdirectory("${I2W_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/i2w")
# add_subdirectory("${CRAWLER_I2W_MSGS_SOURCE_DIR}"
#                  "${CMAKE_CURRENT_BINARY_DIR}/crawler_i2w_msgs")

# add_executable(stm_node
#   src/main.cpp

#   src/core/channelSwitching.cpp
#   src/core/rasterController.cpp
#   src/core/stmi2wNode.cpp
#   src/core/rasterSDK.cpp
#   src/core/UART.cpp

# )
# target_compile_features(stm_node PRIVATE cxx_std_20)
# target_include_directories(stm_node PRIVATE
#   "${CMAKE_CURRENT_SOURCE_DIR}/include")
# target_link_libraries(stm_node PRIVATE
#   i2w::i2w
#   crawler_i2w_msgs::crawler_i2w_msgs
# )

# install(TARGETS stm_node
#   RUNTIME DESTINATION lib/${PROJECT_NAME})


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