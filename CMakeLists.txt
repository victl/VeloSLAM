cmake_minimum_required(VERSION 2.8.11)
project( VeloSlam )
add_definitions(-std=c++11)

#-----> Handling defferent platforms
if(UNIX AND NOT APPLE)
  set(LINUX TRUE)
endif()
#set(ONCAR TRUE)
if(ONCAR)
    add_definitions(-DONCAR)
elseif(LINUX)
    add_definitions(-DLINUX)
elseif(APPLE)
    add_definitions(-DAPPLE)
endif()
##########################
# require Qt5 libraries. #
##########################
#-----> If your Qt is not installed at the standard location(so
#-----> that Cmake couldn't find it), Please set the ${QT5_PATH},
#-----> ${QT5_MODULE_PATH}, {CMAKE_PREFIX_PATH}.
#-----> Similar rules apply to any other libraries that are installed
#-----> at non-standard location.
# Please set the QT5_PATH, QT5_MODULE_PATH, CMAKE_PREFIX_PATH according 
# to your system profile.
if(ONCAR)
  # paths need update
  set(QT5_PATH /opt/Qt5.7)
  set(QT5_MODULE_PATH ${QT5_PATH}/lib/cmake)
  set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${QT5_MODULE_PATH})
elseif(APPLE)
    # if qt is not in standard location, set paths here
elseif(LINUX)
    # same as previous
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

#-----> For .ui files and many other resourse files to show up in your
#-----> project/workspace, add them to ${UI_RES_FILES}
set(UI_RES_FILES pclviewer.ui)
find_package( Qt5Widgets )


##############################################################################
# require Point Cloud Library. If needed, uncomment the find_package() line. #
##############################################################################
if(ONCAR)
  # paths need update
  set(PCL_PATH /usr/local/pcl-1.8.0-rc2)
  set(PCL_MODULE_PATH /usr/local/pcl-1.8.0-rc2/share/pcl-1.8)
  set(CMAKE_PREFIX_PATH ${PCL_MODULE_PATH} ${CMAKE_PREFIX_PATH})
  set(VTK_PATH /usr/local/vtk-7.0.0)
  set(VTK_MODULE_PATH /usr/local/vtk-7.0.0/lib/cmake)
  set(CMAKE_PREFIX_PATH ${VTK_MODULE_PATH} ${CMAKE_PREFIX_PATH})
elseif(APPLE)
    # if pcl/vtk is not in standard location, set paths here
elseif(LINUX)
    # same as previous
endif()

#find_package(PCL 1.7 REQUIRED)
find_package(PCL 1.8 REQUIRED)
if(PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif()

#----> find eigen3
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

#----> find boost
if(ONCAR)
  # paths need update
  set(BOOST_ROOT /usr/local/boost)
  set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${BOOST_ROOT})
  set(Boost_NO_BOOST_CMAKE TRUE)
  set(Boost_NO_SYSTEM_PATHS TRUE)
  set(Boost_LIBRARY_DIRS /usr/local/boost/lib)
elseif(APPLE)
    # if boost is not in standard location, set paths here
elseif(LINUX)
    # same as previous
endif()
if(WIN32 OR APPLE)
  set(Boost_USE_STATIC_LIBS ON)
endif(WIN32 OR APPLE)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED COMPONENTS system thread chrono regex date_time filesystem)
set(ALL_BOOST_LIBRARIES ${Boost_LIBRARIES})

#----> find pcap
find_library(PCAP_LIBRARY pcap DOC "pcap library")
find_path(PCAP_INCLUDE_DIR pcap.h DOC "pcap include directory")
include_directories(${PCAP_INCLUDE_DIR})

# other dependencies
set(deps)

list(APPEND deps
  ${PCAP_LIBRARY}
  ${ALL_BOOST_LIBRARIES}
  )

#----> slam library sources
set(slam_sources)
list(APPEND slam_sources
    vtkPacketFileReader.h
    vtkPacketFileWriter.cxx
    vvPacketSender.cxx
    TransformManager.cxx
    HDLParser.cxx
    INSSource.cxx
    HDLSource.cxx
    HDLManager.cxx
    HDLFrame.cxx
    TimeSolver.cxx
    CoordiTran.cpp
    SmallPacketSender.cxx
    type_defs.h
    type_defs.cxx
    VeloSLAM.h
    TimeLine.h
    )

set(slam_library_name VeloSLAM)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_library(${slam_library_name} SHARED ${slam_sources})
target_link_libraries(${slam_library_name} ${PCAP_LIBRARY} ${ALL_BOOST_LIBRARIES} ${PCL_LIBRARIES} glog)

add_executable(PacketFileSender PacketFileSender.cxx)
target_link_libraries(PacketFileSender LINK_PUBLIC ${slam_library_name})

add_executable(TestINSSender TestINSSender.cxx)
target_link_libraries(TestINSSender LINK_PUBLIC ${slam_library_name})

add_executable(TestINSReader TestINSReader.cxx)
target_link_libraries(TestINSReader LINK_PUBLIC ${slam_library_name})

add_executable(SimpleHDLTester SimpleHDLTester.cxx)
target_link_libraries(SimpleHDLTester LINK_PUBLIC ${slam_library_name} ${slam_main_deps} ${ALL_BOOST_LIBRARIES})
add_executable(EigenTest EigenTestMain.cxx type_defs.cxx)
target_link_libraries(EigenTest ${slam_library_name})

set(slam_main_srcs
    SlamMain.cpp
    pclviewer.cpp
    pclviewer.h)
set(slam_main_deps
    ${PCL_LIBRARIES}
    glog)
add_executable(SlamMain ${slam_main_srcs} ${UI_RES_FILES})
if(APPLE)
    set_target_properties(SlamMain PROPERTIES MACOSX_BUNDLE TRUE)
endif(APPLE)
target_link_libraries(SlamMain LINK_PUBLIC ${slam_main_deps} ${slam_library_name} ${ALL_BOOST_LIBRARIES})
qt5_use_modules(SlamMain Widgets)

add_executable(TestINSFile TestINSFile.cxx)
target_link_libraries(TestINSFile LINK_PUBLIC ${slam_library_name})
add_executable(TestInterpolate TestInterpolateTransformMeasure.cxx)
target_link_libraries(TestInterpolate LINK_PUBLIC ${slam_main_deps} ${slam_library_name} ${ALL_BOOST_LIBRARIES})

add_executable(playground aplayground.cxx)
target_link_libraries(playground LINK_PUBLIC ${slam_main_deps} ${slam_library_name} ${ALL_BOOST_LIBRARIES})
