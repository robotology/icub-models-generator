# Copyright (C) 2013 RobotCub Consortium
# Author: Francesco Romano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

SET(PROJECTNAME iCub2GazeboPIDConverters)
PROJECT(${PROJECTNAME})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

find_package(TinyXML QUIET)
find_package(YARP)
find_package(ICUB)

if(TinyXML_FOUND)
    set(TINYXML_INCLUDES ${TinyXML_INCLUDE_DIRS})
    set(TINYXML_LIBS ${TinyXML_LIBRARIES})
else(TinyXML_FOUND)
    #don't work...?
    set(TINYXML_INCLUDES /usr/local/include)
    set(TINYXML_LIBS /usr/local/lib/libtinyxml.dylib)
endif(TinyXML_FOUND)

set(INCLUDE_DIR include)
#can I specify these two dirs in some other ways?
set(ROBOTINTERFACE_INCLUDE_DIR ../../../main/src/core/robotInterface)
set(DEBUGINTERFACE_INCLUDE_DIR ../../../main/src/libraries/icubmod)


include_directories(${INCLUDE_DIR} ${ROBOTINTERFACE_INCLUDE_DIR} ${DEBUGINTERFACE_INCLUDE_DIR})
include_directories(${TINYXML_INCLUDES} ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

set(SOURCES src/main.cpp src/ICUBConfigParser.cpp)
set(HEADERS ${INCLUDE_DIR}/ICUBConfigParser.h ${INCLUDE_DIR}/Constants.h)

set(ROBOTINTERFACE_INCLUDES ${ROBOTINTERFACE_INCLUDE_DIR}/Param.h ${ROBOTINTERFACE_INCLUDE_DIR}/Types.h ${ROBOTINTERFACE_INCLUDE_DIR}/XMLReader.h ${ROBOTINTERFACE_INCLUDE_DIR}/Robot.h ${ROBOTINTERFACE_INCLUDE_DIR}/Action.h ${ROBOTINTERFACE_INCLUDE_DIR}/Device.h ${ROBOTINTERFACE_INCLUDE_DIR}/CalibratorThread.h)
set(ROBOTINTERFACE_SOURCES ${ROBOTINTERFACE_INCLUDE_DIR}/Action.cpp ${ROBOTINTERFACE_INCLUDE_DIR}/CalibratorThread.cpp ${ROBOTINTERFACE_INCLUDE_DIR}/Device.cpp ${ROBOTINTERFACE_INCLUDE_DIR}/Param.cpp ${ROBOTINTERFACE_INCLUDE_DIR}/Robot.cpp ${ROBOTINTERFACE_INCLUDE_DIR}/Types.cpp ${ROBOTINTERFACE_INCLUDE_DIR}/XMLReader.cpp)


SOURCE_GROUP("Source Files" FILES ${SOURCES})
SOURCE_GROUP("Header Files" FILES ${HEADERS})

add_library(RobotInterface STATIC ${ROBOTINTERFACE_SOURCES} ${DEBUGINTERFACE_INCLUDE_DIR}/debugStream/Debug.cpp)

add_executable(${PROJECTNAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECTNAME} ${TINYXML_LIBS} ${YARP_LIBRARIES} RobotInterface)
