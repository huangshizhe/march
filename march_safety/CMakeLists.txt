cmake_minimum_required(VERSION 2.8.3)
project(march_safety)
add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        march_shared_resources
        roscpp
        roslint
        rostest
        sensor_msgs
        std_msgs
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
)

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "src/*.h"
        )

roslint_cpp(${lintfiles})
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(sourcefiles
        src/SafetyNode.cpp
        include/march_safety/TemperatureSafety.h
        src/TemperatureSafety.cpp
        )

add_library(march_safety ${sourcefiles})

add_executable(march_safety_node src/SafetyNode.cpp src/TemperatureSafety.cpp include/march_safety/TemperatureSafety.h)
add_dependencies(march_safety_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(march_safety_node ${catkin_LIBRARIES})



#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    add_rostest_gtest(${PROJECT_NAME}-test
            test/march_safety.test
            test/TestTemperatureError.cpp
            test/TestNoTemperatureError.cpp
            test/TestTemperatureParameterized.cpp
            )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_safety gtest gmock)

endif ()
