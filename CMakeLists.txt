SET(PROJECT_NAME FivePoint)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CMAKE_VERBOSE_MAKEFILE OFF)

IF( NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++0x")

FIND_PACKAGE(OpenCV REQUIRED)

SET(SOURCE_DIR "src")

SET(SOURCES 
${SOURCE_DIR}/ImageGrabber.cpp
${SOURCE_DIR}/FrameGrabber.cpp
${SOURCE_DIR}/GLWindow2.cpp
${SOURCE_DIR}/HarrisScores.cpp
${SOURCE_DIR}/QuadTree.cpp
${SOURCE_DIR}/ImageGrid.cpp
${SOURCE_DIR}/CvOrb.cpp
${SOURCE_DIR}/FivePoint.cpp
)

SET(LIBS opencv_core opencv_features2d opencv_flann cvd GVars3 GL GLU toontag lapack pthread)

ADD_LIBRARY(fivepoint STATIC ${SOURCES})
TARGET_LINK_LIBRARIES(fivepoint ${LIBS})

# Apps
ADD_EXECUTABLE(main ${SOURCE_DIR}/main.cpp)
TARGET_LINK_LIBRARIES(main fivepoint)

ADD_EXECUTABLE(LiveVideo ${SOURCE_DIR}/LiveVideo.cpp)
TARGET_LINK_LIBRARIES(LiveVideo fivepoint)

# Experiments
ADD_EXECUTABLE(InitExp ${SOURCE_DIR}/InitExp.cpp)
TARGET_LINK_LIBRARIES(InitExp fivepoint)

ADD_EXECUTABLE(RealExp ${SOURCE_DIR}/RealExp.cpp)
TARGET_LINK_LIBRARIES(RealExp fivepoint)

ADD_EXECUTABLE(OutlierExp ${SOURCE_DIR}/OutlierExp.cpp)
TARGET_LINK_LIBRARIES(OutlierExp fivepoint)
