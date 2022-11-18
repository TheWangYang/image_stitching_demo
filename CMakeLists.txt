cmake_minimum_required(VERSION 3.24)

Project(cmake_test)

# 添加c++ 17标准支持
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR F:\\cpp_tools\\tools\\opencv_4_build)
find_package(OpenCV 4.5.3 REQUIRED)

##下面输出opencv信息
# message(STATUS "Opnecv ;ibrary status: ")
message(STATUS "> version: ${OpenCV_VERSION} ")
# message(STATUS "libraries: ${OpenCV_LIBS} ")
# message(STATUS "> include: ${OpenCV_INCLUDE_DIRS}  ")

include_directories(${OpenCV_INCLUDE_DIRS})
# 添加各个子目录
set(CPP_DIR
   ./src
)

# aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/.   PROJECT_SRC)
# 遍历各子目录
foreach(SUB_DIR ${CPP_DIR})
    message(STATUS "---Sub Dir: ${SUB_DIR}")

    file(GLOB SRC "${CMAKE_CURRENT_SOURCE_DIR}/${SUB_DIR}/*.cpp")
    message(STATUS "---all cpp: ${SRC}")

    foreach(CPP ${SRC})
        message(STATUS "---Create Exe From: ${CPP}")

        STRING(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${CPP})
        MESSAGE(" --- FILE_NAME:  ${FILE_NAME}")

        add_executable(${FILE_NAME} ${CPP})
        target_link_libraries(${FILE_NAME} ${OpenCV_LIBS})

    endforeach(CPP)
endforeach(SUB_DIR)
# add_executable(${PROJECT_NAME} ${SRC_LIST})
message(" --------- Create Finished----- \n")
