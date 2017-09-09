cmake_minimum_required (VERSION 3.0)
project (Simplify)

set(CMAKE_BUILD_TYPE "Debug")

if("${CMAKE_BUILD_TYPE}" MATCHES "Debug")
  add_definitions(-DDEBUG)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O0 -fPIC")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -g -O0 -fPIC -D_POSIX_C_SOURCE=200809")
else()
  add_definitions(-DARMA_NO_DEBUG)
  add_definitions(-DNDEBUG)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -fPIC")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -O3 -fPIC -D_POSIX_C_SOURCE=200809")
endif()

# --------------------------------------------------
	# Tell CMake to run moc when necessary:
	set(CMAKE_AUTOMOC ON)
	# As moc files are generated in the binary dir, tell CMake
	# to always look for includes there:
	set(CMAKE_INCLUDE_CURRENT_DIR ON)

	# Widgets finds its own dependencies (QtGui and QtCore).
	find_package(Qt5Widgets)
	#  find_package(Qt5Declarative)

	# The Qt5Widgets_INCLUDES also includes the include directories for
	# dependencies QtCore and QtGui
	include_directories(${Qt5Widgets_INCLUDES})

	# We need add -DQT_WIDGETS_LIB when using QtWidgets in Qt 5.
	add_definitions(${Qt5Widgets_DEFINITIONS})
# --------------------------------------------------
#opencv should be built first!
message("OPENCV_BRANCH=$ENV{OPENCV_BRANCH}")
string( SUBSTRING "$ENV{OPENCV_BRANCH}" 0 4 OPENCV_MAJOR)
message("OPENCV_MAJOR=${OPENCV_MAJOR}")

if(ANDROID_NDK)
	#if(${APP_ABI} STREQUAL "armeabi-v7a" )
		#openCV 3.2.0 SDK by "make install"
		#CV_MAJOR_VERSION==3
		set(OpenCV_DIR $ENV{OPENCV_OUT}/${APP_ABI}/sdk/native/jni)
		include( "${OpenCV_DIR}/OpenCVConfig.cmake" )
		message("++++OpenCV_DIR:${OpenCV_DIR}")
		FIND_PACKAGE( OpenCV REQUIRED)
		if(OpenCV_FOUND)
			message("found")
			message( "OpenCV_INCLUDE_DIRS:${OpenCV_INCLUDE_DIRS}" )
			message( "OpenCV_INCLUDES:${OpenCV_INCLUDES}")
			message( "OpenCV_LIBRARIES:${OpenCV_LIBRARIES}")
			message( "OpenCV_LIBS:${OpenCV_LIBS}")
			message( "OpenCV_DEFINITIONS:${OpenCV_DEFINITIONS}")
			add_definitions(-DUSE_OPENCV=${USE_OPENCV})
			set(OPENCV_BRANCH $ENV{OPENCV_BRANCH})
			include_directories(${OpenCV_INCLUDE_DIRS})
		else()
			message(FATAL_ERROR "openCV not foud")
		endif()
	#endif()
else()
#opencv should be built first!
	set(OpenCV_DIR $ENV{OPENCV_OUT})
	#set(OpenCV_DIR "/home/thomas/opencv/build_2.4.13/debug")
	#set(OpenCV_DIR "/home/thomas/build/bcv-x86.gl/out/opencv/2.4.x-static/x86_64")
	#set(OpenCV_DIR "/home/thomas/opencv/build_3.2.0/debug")
	#add_definitions(-DOPENCV_VER=2)
	message("++++OpenCV_DIR:${OpenCV_DIR}")
	FIND_PACKAGE( OpenCV REQUIRED)
	if(OpenCV_FOUND)
		message("found")
		message( "OpenCV_INCLUDE_DIRS:${OpenCV_INCLUDE_DIRS}" )
		message( "OpenCV_INCLUDES:${OpenCV_INCLUDES}")
		message( "OpenCV_LIBRARIES:${OpenCV_LIBRARIES}")
		message( "OpenCV_LIBS:${OpenCV_LIBS}")
		message( "OpenCV_DEFINITIONS:${OpenCV_DEFINITIONS}")
		add_definitions(-DUSE_OPENCV=${USE_OPENCV})
		set(OPENCV_BRANCH $ENV{OPENCV_BRANCH})
		include_directories(${OpenCV_INCLUDE_DIRS})
	else()
		message(FATAL_ERROR "openCV not foud")
	endif()

	message("*** OpenCV_LIBS:${OpenCV_LIBS}")
	message("*** OpenCV_LIBRARIES:${OpenCV_LIBRARIES}")
endif()

	
add_executable(Simplify Simplify.cpp)
qt5_use_modules(Simplify Widgets  OpenGL Test Concurrent)
target_link_libraries(Simplify
		${OpenCV_LIBS}
	)
