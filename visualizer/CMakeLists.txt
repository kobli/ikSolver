cmake_minimum_required (VERSION 2.8)
project(visualizer)
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/extlibs)
FIND_PACKAGE(Irrlicht REQUIRED)

file(GLOB CPP_SOURCES
	"${PROJECT_SOURCE_DIR}/src/*.cpp"
	)
include_directories(
	${PROJECT_SOURCE_DIR}/include
	${IRRLICHT_INCLUDE_DIR}
	)
set(PLATFORMS lin_64 win_gcc_32)
set_property(CACHE PLATFORM PROPERTY STRINGS ${PLATFORMS})
set(CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/extlibs")
link_directories(
	/usr/X11R6/lib64
	)


add_executable(${PROJECT_NAME} ${CPP_SOURCES})

set(LINUX_ONLY_LIBS
	GL
	Xxf86vm
	Xext
	X11
	Xcursor
	dl
	)

set(WINDOWS_ONLY_LIBS
	ws2_32
	winmm
	gdi32
	opengl32
	winmm
	gdi32
	ws2_32
	)

set(GENERAL_LIBS
	Irrlicht
	ikSolver
	#BulletDynamics
	#BulletCollision
	#LinearMath
	)

if(${PLATFORM} MATCHES "lin_64")
	set(TARGET_OS "Linux")
elseif(${PLATFORM} MATCHES "win_gcc_32")
	set(TARGET_OS "Windows")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_IRR_STATIC_LIB_ -static -static-libgcc -static-libstdc++")
else()
	message(FATAL_ERROR "PLATFORM variable not set. Possible values: ${PLATFORMS}")
endif()

if(${TARGET_OS} MATCHES "Linux")
	set(SYSTEM_SPECIFIC_LIBS ${LINUX_ONLY_LIBS})
elseif(${TARGET_OS} MATCHES "Windows")
	set(SYSTEM_SPECIFIC_LIBS ${WINDOWS_ONLY_LIBS})
endif()

target_link_libraries(${PROJECT_NAME}
	${GENERAL_LIBS}
	${SYSTEM_SPECIFIC_LIBS}
	${IRRLICHT_LIBRARY}
	)
