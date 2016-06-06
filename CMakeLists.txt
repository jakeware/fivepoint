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

SET(LIBS opencv_core opencv_features2d opencv_flann cvd GVars3 GL GLU toontag lapack boost_system boost_thread pthread)

# Apps
ADD_EXECUTABLE(main ${SOURCE_DIR}/main.cpp ${SOURCES})
TARGET_LINK_LIBRARIES(main ${LIBS})

ADD_EXECUTABLE(LiveVideo ${SOURCE_DIR}/LiveVideo.cpp ${SOURCES})
TARGET_LINK_LIBRARIES(LiveVideo ${LIBS})

# Experiments
ADD_EXECUTABLE(InitExp ${SOURCE_DIR}/InitExp.cpp ${SOURCES})
TARGET_LINK_LIBRARIES(InitExp ${LIBS})

ADD_EXECUTABLE(RealExp ${SOURCE_DIR}/RealExp.cpp ${SOURCES})
TARGET_LINK_LIBRARIES(RealExp ${LIBS})

#ADD_EXECUTABLE(OutlierExp ${SOURCE_DIR}/OutlierExp.cpp ${SOURCES})
#TARGET_LINK_LIBRARIES(OutlierExp ${LIBS})
