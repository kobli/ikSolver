cmake_minimum_required (VERSION 2.8)
project(ikSolver)

file(GLOB CPP_SOURCES
	"${PROJECT_SOURCE_DIR}/src/*.cpp"
	)
include_directories(
	${PROJECT_SOURCE_DIR}/include
	)

add_library(${PROJECT_NAME} ${CPP_SOURCES})
