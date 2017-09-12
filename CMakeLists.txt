CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
project(shark)

option(USE_ZED "Use ZED for img feed instead of sim" OFF)
option(USE_GUST "Use GUST alongside for sim andor controller" ON)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)


SET(EXECUTABLE_OUTPUT_PATH ".")

##OCV
SET(VERSION_REQ_OCV "2.4")
find_package(OpenCV ${VERSION_REQ_OCV} REQUIRED)

if(USE_GUST)

##BOOST
SET(VERSION_REQ_BOOST "1.49")
find_package(Boost ${VERSION_REQ_BOOST} REQUIRED)

add_definitions(-DGUST)

endif()

if(USE_ZED)
add_definitions(-DZED)
SET(VERSION_REQ_CUDA "8.0")
find_package(ZED 2.0 REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})

##For Jetson, OpenCV4Tegra is based on OpenCV2.4
exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
   

SET(VERSION_REQ_CUDA "")

unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
message ("CUDA_USE_STATIC_CUDA_RUNTIME : ${CUDA_USE_STATIC_CUDA_RUNTIME}")
##to prevent from opencv_dep_cudart dependencies error...
## cmake with -DCUDA_USE_STATIC_CUDA_RUNTIME=false can also be called.



find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIR})
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${CUDA_LIBRARY_DIRS})

SET(SRC_FOLDER src)
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")

ADD_EXECUTABLE(shark ${SRC_FILES})
add_definitions(-std=c++0x -g -O3)

if(USE_ZED AND USE_GUST)
	TARGET_LINK_LIBRARIES(shark
				${ZED_LIBRARIES}
				${OpenCV_LIBRARIES}
				${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
				${Boost_LIBRARIES}
				)
elseif(USE_GUST)
	TARGET_LINK_LIBRARIES(shark
				${OpenCV_LIBRARIES}
				${Boost_LIBRARIES}
			)
elseif(USE_ZED)
	TARGET_LINK_LIBRARIES(shark
				${ZED_LIBRARIES}
				${OpenCV_LIBRARIES}
				${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
				)
endif()