# Install script for directory: /home/koko/Desktop/Localization_Jackal/src/jackal_simulator/jackal_gazebo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/koko/Desktop/Localization_Jackal/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/koko/Desktop/Localization_Jackal/build/jackal_simulator/jackal_gazebo/catkin_generated/installspace/jackal_gazebo.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jackal_gazebo/cmake" TYPE FILE FILES
    "/home/koko/Desktop/Localization_Jackal/build/jackal_simulator/jackal_gazebo/catkin_generated/installspace/jackal_gazeboConfig.cmake"
    "/home/koko/Desktop/Localization_Jackal/build/jackal_simulator/jackal_gazebo/catkin_generated/installspace/jackal_gazeboConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jackal_gazebo" TYPE FILE FILES "/home/koko/Desktop/Localization_Jackal/src/jackal_simulator/jackal_gazebo/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jackal_gazebo" TYPE DIRECTORY FILES
    "/home/koko/Desktop/Localization_Jackal/src/jackal_simulator/jackal_gazebo/launch"
    "/home/koko/Desktop/Localization_Jackal/src/jackal_simulator/jackal_gazebo/Media"
    "/home/koko/Desktop/Localization_Jackal/src/jackal_simulator/jackal_gazebo/worlds"
    )
endif()

