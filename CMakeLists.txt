cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

project(MapConstructor)

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

if(WIN32)
  if(NOT CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX d)
  endif()
endif()

set(CMAKE_PREFIX_PATH "C:/Program Files" CACHE PATH "where to find packages")

set(CMAKE_GENERATOR_PLATFORM x64)

set(MKL_INCLUDE_DIR  "" CACHE PATH "include directory of MKL")

if(WIN32)
    set(CMAKE_CXX_FLAGS "/openmp")
endif()
if(UNIX)
	set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-invalid-offsetof -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-reorder")
	set(CMAKE_CXX_FLAGS_DEBUG "-g")
	set(CMAKE_CXX_FLAGS_RELEASE "-O2")
endif()

if(WIN32)
	set(Boost_USE_STATIC_LIBS        ON)
	set(Boost_USE_MULTITHREADED      ON)
endif()
find_package(Boost REQUIRED COMPONENTS thread timer)

find_package(Eigen3 3.3 REQUIRED)
find_package(GLUT)
find_package(GLEW)
find_package(EXPAT)
find_package(Springhead)
find_package(Scenebuilder)
find_package(g2o)
find_package(SuiteSparse)
find_library(EXPAT_LIBRARY
  NAMES expat
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${EXPAT_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${G2O_INCLUDE_DIR})
include_directories(${MKL_INCLUDE_DIR})

set(MAP_OPTIMIZER_HEADERS
  "include/base.h"
  "include/converter.h"
  "include/gauss_newton.h"
  "include/g2o_file_io.h"
  "include/g2o_optimizer.h"
  "include/map_direction_optimizer.h"
  "include/map_loader.h"
  "include/map_saver.h"
  "include/provisional_task.h"
  "include/prox_optimizer.h"
  "include/scan_matcher.h"
  "include/util.h"
)

set(MAP_OPTIMIZER_SOURCES
  "src/base.cpp"
  "src/converter.cpp"
  "src/gauss_newton.cpp"
  "src/g2o_file_io.cpp"
  "src/g2o_optimizer.cpp"
  "src/main.cpp"
  "src/map_direction_optimizer.cpp"
  "src/map_loader.cpp"
  "src/map_saver.cpp"
  "src/prox_optimizer.cpp"
  "src/scan_matcher.cpp"
)

set(MAP_OPTIMIZER_CONFIGS
  "conf/platform.xml"
  "conf/nc2021.xml"
  "conf/nc2022.xml"
  "conf/nc2023.xml"
  "conf/nc2023ex.xml"
  "conf/ku2022.xml"
  "conf/ku2023.xml"
  "conf/ku2024.xml"
  "conf/geo_convert.xml"
  "conf/map_convert.xml"
)

source_group("Header Files"	FILES ${MAP_OPTIMIZER_HEADERS})
source_group("Source Files"	FILES ${MAP_OPTIMIZER_SOURCES})
source_group("Config Files"	FILES ${MAP_OPTIMIZER_CONFIGS})

add_executable(MapConstructor
  ${MAP_OPTIMIZER_CONFIGS}
  ${MAP_OPTIMIZER_HEADERS}
  ${MAP_OPTIMIZER_SOURCES}
)
 
target_link_libraries(MapConstructor ${Boost_LIBRARIES})
target_link_libraries(MapConstructor Springhead::Springhead)
target_link_libraries(MapConstructor Scenebuilder)
target_link_libraries(MapConstructor GLUT::GLUT)
target_link_libraries(MapConstructor GLEW::glew)
target_link_libraries(MapConstructor ${EXPAT_LIBRARIES})
target_link_libraries(MapConstructor g2o::freeglut_minimal
									g2o::g2o_ceres_ad
									g2o::stuff
									g2o::opengl_helper
									g2o::core
									g2o::g2o_cli_library
									g2o::types_slam2d
									g2o::solver_eigen)
target_link_libraries(MapConstructor ${G2O_LIBRARIES})

#install(TARGETS MapConstructor
#  EXPORT        MapConstructorExport
#  LIBRARY       DESTINATION lib
#  INCLUDES      DESTINATION include/MapConstructor 
#  PUBLIC_HEADER DESTINATION include/MapConstructor 
#)

#install(EXPORT MapConstructorExport
#  FILE         MapConstructorConfig.cmake
#  DESTINATION  cmake
#)

#install(FILES ${MAP_OPTIMIZER_EXTERNAL_LIBS}	DESTINATION lib)

#install(FILES ${MAP_OPTIMIZER_HEADERS}			DESTINATION include/MapConstructor)

## print all variables
#get_cmake_property(_variableNames VARIABLES)
#foreach(_variableName ${_variableNames})
#	message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
