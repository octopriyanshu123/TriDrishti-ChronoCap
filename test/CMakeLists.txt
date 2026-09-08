cmake_minimum_required(VERSION 3.16)

# i2w-specific nodes: the "glue" layer between the schema-agnostic
# chrono_cap library and this project's actual message types/i2w runtime.

# add_executable(pub pub.cpp)
# target_link_libraries(pub PRIVATE pthread)

# add_executable(sub sub.cpp)
# target_link_libraries(sub PRIVATE pthread nlohmann_json::nlohmann_json)

# add_executable(i2wRecorder i2wRecorder.cpp)
# target_link_libraries(i2wRecorder PRIVATE chrono_cap pthread nlohmann_json::nlohmann_json)

# add_executable(i2wReplayer i2wReplayer.cpp)
# target_link_libraries(i2wReplayer PRIVATE chrono_cap pthread)

# target_include_directories(i2wRecorder PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
# target_include_directories(i2wReplayer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})


# basic Test 
add_executable(generate_and_record chrono_cap_basic/generate_and_record.cpp)
target_link_libraries(generate_and_record PRIVATE chrono_cap pthread)
target_include_directories(generate_and_record PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(replay_and_print chrono_cap_basic/replay_and_print.cpp)
target_link_libraries(replay_and_print PRIVATE chrono_cap pthread)
target_include_directories(replay_and_print PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})