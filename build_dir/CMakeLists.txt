cmake_minimum_required(VERSION 3.5)

project(Study_Forward_Kinematics CXX C)

set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE false)
set(CMAKE_CXX_FLAGS_DEBUG "-g -Winline -Wall -O0")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage")
set(EXECUTABLE_OUTPUT_PATH bin)
set(INCLUDE_DIR include)
set(SOURCE_DIR src)
set(INCLUDE_DIRS ${SOURCE_DIR} ${INCLUDE_DIR})

set(USE_BOOST 	0)
set(USE_CV 		0)
set(USE_EIGEN3 	1)
set(USE_ICE		0)
set(USE_ODE 	0)
set(USE_SSH 	0)

file(GLOB CXX_SOURCE_FILES "${SOURCE_DIR}/*.cpp")
file(GLOB C_SOURCE_FILES "${SOURCE_DIR}/*.c")
set(SOURCE_FILES ${CXX_SOURCE_FILES} ${C_SOURCE_FILES})
set(TARGET_NAME ${PROJECT_NAME}.out)

if(USE_ODE)
	if(APPLE)
		set(ODE_TOP_DIRECTORIES /usr/local/Cellar/ode-drawstuff/0.13/)
	elseif(UNIX)
		set(ODE_TOP_DIRECTORIES /usr/local)
	endif()
	set(ODE_INCLUDE_DIR ${ODE_TOP_DIRECTORIES}/include)
	set(ODE_LIBRARIES_DIR ${ODE_TOP_DIRECTORIES}/lib)
	if(EXISTS ${ODE_INCLUDE_DIR} AND EXISTS ${ODE_LIBRARIES_DIR})
		message(" ---- Found Ode ${ODE_INCLUDE_DIR}")
		include_directories(${ODE_INCLUDE_DIR})
		file(GLOB ODE_LIBRARIES "${ODE_LIBRARIES_DIR}/*.a")
	else()
		message(" ---- Not Found Eigen3 Top Directories")
	endif()
endif(USE_ODE)

if(USE_SSH)
	if(APPLE)
		set(LIBSSH_TOP_DIR /usr/local)
	elseif(UNIX)
		set(LIBSSH_TOP_DIR /usr)
	endif()
	set(libssh_INCLUDE_DIRS ${LIBSSH_TOP_DIR}/include)
	set(libssh_LIBRARIES_DIR ${LIBSSH_TOP_DIR}/lib)
	if(EXISTS ${libssh_INCLUDE_DIRS} AND EXISTS ${libssh_LIBRARIES_DIR})
		message(" ---- Found libssh ${libssh_INCLUDE_DIRS}")
		include_directories(${libssh_INCLUDE_DIRS})
		file(GLOB libssh_LIBRARIES "${libssh_LIBRARIES_DIR}/libssh*.dylib")
	else()
		message(" ---- Not Found libssh Top Directories")
	endif()
endif(USE_SSH)

if(USE_CV)
	find_package(OpenCV REQUIRED)
	if(OpenCV_FOUND)
		message(" ---- Found OpenCV ${OpenCV_INCLUDE_DIRS}")
		include_directories(${OpenCV_INCLUDE_DIRS})
	else(OpenCV_FOUND)
		message(" ---- Not Found OpenCV")
	endif(OpenCV_FOUND)
endif(USE_CV)

if(USE_BOOST)
	find_package(Boost	REQUIRED COMPONENTS thread system)
	if(Boost_FOUND)
		message(" ---- Found Boost ${Boost_INCLUDE_DIRS}")
		include_directories(${Boost_INCLUDE_DIRS})
	else(Boost_FOUND)
		message(" ---- Not Found Boost")
	endif(Boost_FOUND)
endif(USE_BOOST)

if(USE_EIGEN3)
	if(APPLE)
		find_package(Eigen3	REQUIRED COMPONENTS Dense Core)
		if(Eigen3_FOUND)
			message(" ---- Found Eigen3 ${EIGEN3_INCLUDE_DIRS}")
			include_directories(${EIGEN3_INCLUDE_DIRS})
		else(Eigen3_FOUND)
			message(" ---- Not Found Eigen3")
		endif(Eigen3_FOUND)
	elseif(UNIX)
		if(EXISTS $ENV{EIGEN3_INCLUDE_DIR})
			message(" ---- Used Unix Eigen3 Dir $ENV{EIGEN3_INCLUDE_DIR}")
			include_directories($ENV{EIGEN3_INCLUDE_DIR})
		else()
			message(" ---- Not Defind of EIGEN3_INCLUDE_DIRS")
		endif()
	endif()
endif(USE_EIGEN3)

if(USE_ICE)
	find_package(Ice	REQUIRED COMPONENTS Ice IceBox Ice++11)
	if(Ice_FOUND)
		message(" ---- Found Ice ${Ice_INCLUDE_DIRS}")
		include_directories(${Ice_INCLUDE_DIRS})
		add_definitions(-DICE_CPP11_MAPPING)
	else(Ice_FOUND)
		message(" ---- Not Found Ice")
	endif(Ice_FOUND)
endif(USE_ICE)

include_directories(${INCLUDE_DIRS})

add_executable(${TARGET_NAME} ${SOURCE_FILES})

target_link_libraries(${TARGET_NAME} stdc++ pthread m)
target_link_libraries(${TARGET_NAME} python2.7)

if(USE_ODE)
	if(APPLE)
		target_link_libraries(${TARGET_NAME} ${ODE_LIBRARIES} "-framework AGL -framework OpenGL -framework GLUT")
	elseif(UNIX)
		target_link_libraries(${TARGET_NAME} ${ODE_LIBRARIES} "ode -ldrawstuff -lGL -lGLU -lglut -lX11")
	endif()
endif(USE_ODE)

if(USE_SSH)
	target_link_libraries(${TARGET_NAME} ${libssh_LIBRARIES})
endif(USE_SSH)

if(USE_CV)
	target_link_libraries(${TARGET_NAME} ${OpenCV_LIBRARIES})
endif(USE_CV)

if(USE_BOOST)
	target_link_libraries(${TARGET_NAME} ${Boost_LIBRARIES} Boost::thread)
endif(USE_BOOST)

if(USE_ICE)
	target_link_libraries(${TARGET_NAME} ${Ice_LIBRARIES})
endif(USE_ICE)

