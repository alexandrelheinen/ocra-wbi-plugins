# Copyright (C) 2014 CoDyCo
# Author: Darwin Lau, MingXing Liu, Ryan Lober
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or any later version.

cmake_minimum_required(VERSION 2.8.11)
SET(PROJECTNAME ISIRWholeBodyController)
PROJECT(${PROJECTNAME})

set(CMAKE_CXX_FLAGS "-fpermissive")

find_package(YARP REQUIRED)
find_package(YCM REQUIRED)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${YCM_MODULE_PATH} ${CMAKE_MODULE_PATH})

find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)
find_package(modHelp REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)


# find_package(ocraWbiPlugins REQUIRED)
# find_package(taskSequences REQUIRED)

find_package(PkgConfig REQUIRED)
PKG_CHECK_MODULES(PC_WLQP_CONTROL wLQP-Control REQUIRED)
PKG_CHECK_MODULES(PC_OCRA_FRAMEWORK ocra_framework REQUIRED)

# new "data" installation - available with yarp2.4
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/ISIRWholeBodyController/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})



INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${ocraWbiPlugins_SOURCE_DIR}/include
                    ${taskSequences_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${yarpWholeBodyInterface_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${EIGEN3_INCLUDE_DIR}/unsupported
                    ${BOOST_INCLUDE_DIRS}
                    ${PC_WLQP_CONTROL_INCLUDE_DIRS}
                    ${PC_OCRA_FRAMEWORK_INCLUDE_DIRS}
                    ${modHelp_INCLUDE_DIRS}/include
                    ${Boost_INCLUDE_DIR}
                    )


LINK_DIRECTORIES(${PC_WLQP_CONTROL_LIBRARY_DIRS}
                 ${PC_OCRA_FRAMEWORK_LIBRARY_DIRS}
                 ${ocraWbiPlugins_SOURCE_DIR}/src
                 ${taskSequences_SOURCE_DIR}/src
                 ${taskSequences_SOURCE_DIR}/src/sequences
                 )

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})


TARGET_LINK_LIBRARIES(${PROJECTNAME} ocraWbiPlugins
                                     taskSequences
                                     ${YARP_LIBRARIES}
                                     ${ICUB_LIBRARIES}
                                     ${iDynTree_LIBRARIES}
                                     ${yarpWholeBodyInterface_LIBRARIES}
                                     ${modHelp_LIBRARIES}
                                     ${PC_WLQP_CONTROL_LIBRARIES}
                                     ${PC_OCRA_FRAMEWORK_LIBRARIES}
                                     ${Boost_LIBRARIES}
                                     )


INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_subdirectory(app)
