# Install script for directory: /home/ameyaranade/Desktop/603-robotics/project1/task2/catkin_ws/src/P1D2_ameya_ranade

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ameyaranade/Desktop/603-robotics/project1/task2/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ameyaranade/Desktop/603-robotics/project1/task2/catkin_ws/build/P1D2_ameya_ranade/catkin_generated/installspace/P1D2_ameya_ranade.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/P1D2_ameya_ranade/cmake" TYPE FILE FILES
    "/home/ameyaranade/Desktop/603-robotics/project1/task2/catkin_ws/build/P1D2_ameya_ranade/catkin_generated/installspace/P1D2_ameya_ranadeConfig.cmake"
    "/home/ameyaranade/Desktop/603-robotics/project1/task2/catkin_ws/build/P1D2_ameya_ranade/catkin_generated/installspace/P1D2_ameya_ranadeConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/P1D2_ameya_ranade" TYPE FILE FILES "/home/ameyaranade/Desktop/603-robotics/project1/task2/catkin_ws/src/P1D2_ameya_ranade/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/P1D2_ameya_ranade" TYPE PROGRAM FILES "/home/ameyaranade/Desktop/603-robotics/project1/task2/catkin_ws/build/P1D2_ameya_ranade/catkin_generated/installspace/P1D2_ameya_ranade.py")
endif()

