cmake_minimum_required(VERSION 3.0.2)
project(exchanger)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        cv_bridge
        nodelet
        pluginlib
        dynamic_reconfigure
        tf
        )

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/dynamic.cfg
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp cv_bridge dynamic_reconfigure nodelet
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
        include ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp include/exchanger/header.h)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )