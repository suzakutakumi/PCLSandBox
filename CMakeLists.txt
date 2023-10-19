cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(PROJECT_NAME "sandbox")
project(${PROJECT_NAME})

add_executable(${PROJECT_NAME}
    main.cpp
    PointCloud.cpp
    )

find_package(PCL 1.8 REQUIRED)

message("*************find package fin")

if( PCL_FOUND )
    message("*************PCL_FOUND")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
endif()

message("*************PCL DONE")
