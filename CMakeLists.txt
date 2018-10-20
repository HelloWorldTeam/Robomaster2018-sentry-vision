cmake_minimum_required(VERSION 3.5)
project(RoboMasterShaoBing_new)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(OpenCV_DIR "/usr/local/opencv2_4_13/share/OpenCV")
find_package(OpenCV 2.4.13 REQUIRED)
message(STATUS "opencv version " ${OpenCV_VERSION})

add_definitions(-D__SHOWIMG__)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/MVCamera/include
        ${PROJECT_SOURCE_DIR}/RoboMasterMarkerDetector3/include
        ${PROJECT_SOURCE_DIR}/RoboMasterMarkerDetector3/thirdpart
        ${PROJECT_SOURCE_DIR}/RoboMasterMarkerDetector3/thirdpart/KCFTracker
)

set(SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/src/RemoteControl.cpp
        ${PROJECT_SOURCE_DIR}/src/RoboMasterProcess.cpp
        ${PROJECT_SOURCE_DIR}/src/can.cpp
        ${PROJECT_SOURCE_DIR}/MVCamera/src/MVCamera.cpp
        ${PROJECT_SOURCE_DIR}/RoboMasterMarkerDetector3/src/MarkerParams.cpp
        ${PROJECT_SOURCE_DIR}/RoboMasterMarkerDetector3/src/MarkerSensor.cpp
        ${PROJECT_SOURCE_DIR}/RoboMasterMarkerDetector3/thirdpart/KCFTracker/kcftracker.cpp
        ${PROJECT_SOURCE_DIR}/RoboMasterMarkerDetector3/thirdpart/KCFTracker/fhog.cpp
        )

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

add_executable(main main.cpp ${SOURCE_FILES})
if( ${ARCHITECTURE} STREQUAL "x86_64" )
    target_link_libraries(main
            ${OpenCV_LIBS}
            ${PROJECT_SOURCE_DIR}/MVCamera/lib/libMVSDK.so
            -larmadillo
            -lpthread)
else()
    target_link_libraries(main
            ${OpenCV_LIBS}
            ${PROJECT_SOURCE_DIR}/MVCamera/lib_arm/libMVSDK.so
            -larmadillo
            -lpthread)
endif()
