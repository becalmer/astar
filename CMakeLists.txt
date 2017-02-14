cmake_minimum_required(VERSION 2.8)

project(AStarDemo)

message(STATUS "This is binary dir: " ${PROJECT_BINARY_DIR})
message(STATUS "This is source dir: " ${PROJECT_SOURCE_DIR})

#定义目标，一般与所在目录同名
set(TARGET "AStarDemo")

#原文件路径
#set(SOURCE_DIR "${PROJECT_SOURCE_DIR}/${TARGET}")

#头文件路径
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

#添加源文件
#aux_source_directory(${SOURCE_DIR} SOURCES)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

ADD_DEFINITIONS("-g")

#目标可执行程序
ADD_EXECUTABLE(${TARGET} main.cpp astar.cpp)
