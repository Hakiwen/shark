CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
project(shark)


#### addcomp ##################################################################
#    Usage : addcomp(name ON|OFF description TARGETS target1 target2 DEFINES define1 define2)
#    name   - name of feature/component
#    ON|OFF - is this component on or off by default
#    description - a simple description of the component
#    TARGETS     - a list of targets to which to add these components to
#    DEFINES     - in Addition to -DNAME and -DHAVE_NAME what defines
#                   need to be added when this component is included in a target
#    Outputs:
#    ${target}_COMPONENTS - appends component to this variable
#    ${name}_ONOFF        - indicates whether this component is on or off by default
#    WITH_${name}         - if this component is supposed to be on
#    ${name}_DEFINES      - list additional #defines if component is on
#
#    .This macro adds the component with name "name" to the list of components
#    to each target listed after TARGETS. By adding a component to a target we mean
#    it simply adds it to the ${target}_COMPONENTS list
#    if the component is specified to be on by default
MACRO(addcomp)
    parse_arguments(addcomp "TARGETS;DEFINES" "" ${ARGN})
    list(GET addcomp_DEFAULT_ARGS 0 addcomp_NAME)
    list(GET addcomp_DEFAULT_ARGS 1 addcomp_ONOFF)
    list(GET addcomp_DEFAULT_ARGS 2 addcomp_DESC)
    set(WITH_${addcomp_NAME} ${addcomp_ONOFF} CACHE BOOL "${addcomp_DESC}"  )
    if(WITH_${addcomp_NAME})
        foreach(target ${addcomp_TARGETS})
            list(APPEND ${target}_COMPONENTS ${addcomp_NAME})
            list(APPEND ${addcomp_NAME}_DEFINES    ${addcomp_DEFINES})
        endforeach()
    endif()
ENDMACRO()

MACRO(adddbh)
    foreach(dbhfile ${ARGN})
        if(${dbhfile}_deps)
            SET_SOURCE_FILES_PROPERTIES(${dbhfile} PROPERTIES OBJECT_DEPENDS ${${dbhfile}_deps} HEADER_FILE_ONLY TRUE)
            #message("${dbhfile} depends on ${${dbhfile}_deps}")
        endif()
    endforeach()
ENDMACRO()

set(WITH_ZED ON)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)


SET(EXECUTABLE_OUTPUT_PATH ".")

##OCV
SET(VERSION_REQ_OCV "2.4")
find_package(OpenCV ${VERSION_REQ_OCV} REQUIRED)

if(WITH_ZED)
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

if(WITH_ZED)
	TARGET_LINK_LIBRARIES(shark
				${ZED_LIBRARIES}
				${OpenCV_LIBRARIES}
				${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
				)
else()
	TARGET_LINK_LIBRARIES(shark
				${OpenCV_LIBRARIES}
			)
endif()