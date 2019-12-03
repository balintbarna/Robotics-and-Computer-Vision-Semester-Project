cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(template_matching)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV REQUIRED EXACT 3.2.0 )

find_package(COVIS HINTS /home/student/Workspace/covis/build)
add_definitions(${COVIS_DEFINITIONS})
include_directories(${COVIS_INCLUDE_DIRS})

include_directories(include)

find_package(VTK REQUIRED NO_MODULE)
include(${VTK_USE_FILE})

add_executable(linemod src/linemod.cpp )
target_link_libraries(linemod ${COVIS_LIBRARIES} )

add_executable(create_templates_cad src/create_templates_cad.cpp)
target_link_libraries(create_templates_cad ${COVIS_LIBRARIES})

