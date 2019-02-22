cmake_minimum_required(VERSION 2.8)
project( FTSensors )

if(NOT CMAKE_COMPILER_IS_GNUCXX)
	message(ERROR "The compiler ${CMAKE_CXX_COMPILER} is not a GNU C++ compiler. Please use a GNU C++ compiler.")
endif()

if (CMAKE_VERSION VERSION_LESS "3.1")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
	set(CMAKE_CXX_STANDARD 11)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(WIN32)
	SET(EIGEN3_CMAKE_MODULE "C:/Program Files (x86)/Eigen3/share/eigen3/cmake" CACHE PATH "The directory of the Eigen3 cmake module" FORCE)
	set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} ${EIGEN3_CMAKE_MODULE}")

	#required to suppress the boost warning
	add_definitions(-D _WIN32_WINNT=0x0501)
endif()

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        SET(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/bin CACHE PATH "The directory to use for installing the library" FORCE) 
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

SET(LIBRARY_OUTPUT_PATH ${CMAKE_INSTALL_PREFIX}/lib)

find_package(Boost 1.6 REQUIRED COMPONENTS system thread atomic chrono timer)
find_package(Eigen3 REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

add_library(FTSensors STATIC src/FTSensors.cpp)

if(WIN32)
	target_link_libraries(FTSensors ${Boost_LIBRARIES} ws2_32)
else()
	target_link_libraries(FTSensors ${Boost_LIBRARIES})
endif()

#####################################################################
#Build test app
#####################################################################

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/test/bin)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/test/include)

link_directories(${CMAKE_INSTALL_PREFIX}/lib)

add_executable( ATI_NetBox_FT_Sensor ${PROJECT_SOURCE_DIR}/test/src/ATI_NetBox_FT_Sensor.cpp )
add_dependencies( ATI_NetBox_FT_Sensor 
	FTSensors
)

if(WIN32)
	target_link_libraries(ATI_NetBox_FT_Sensor FTSensors ${Boost_LIBRARIES} ws2_32)
else()
	target_link_libraries(ATI_NetBox_FT_Sensor FTSensors ${Boost_LIBRARIES})
endif()