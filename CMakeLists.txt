cmake_minimum_required (VERSION 3.16)

project ("hw6 project" LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

configure_file(
	"${CMAKE_CURRENT_LIST_DIR}/CMakeIn"
	"${CMAKE_CURRENT_LIST_DIR}/CMakeIn.h"
)

# dependencies
set(THIRD_PARTY_DIR "${CMAKE_CURRENT_LIST_DIR}/3rd")
include_directories(${THIRD_PARTY_DIR})

add_subdirectory("${THIRD_PARTY_DIR}/freeglut")
include_directories("${THIRD_PARTY_DIR}/freeglut/include")

add_subdirectory("${THIRD_PARTY_DIR}/shapeLib")

set(TARGET_NAME "hw6_lib")
add_library(
	${TARGET_NAME}
	"Geometry.cpp"
	"QuadTree.cpp"
	"QuadTreeTest.cpp"
	"RTree.cpp"
	
 "Hilbert.h"  "Hilbert.cpp" "BPlusTreeTest.cpp" "btree.h" "btree.cpp"  "RTreeTest.cpp")
target_link_libraries(
	${TARGET_NAME}
	PUBLIC "freeglut_static"
)

set(TARGET_NAME "hw6_app")
add_executable(
	${TARGET_NAME}
	"hw6.cpp"
"Hilbert.h"  "Hilbert.cpp" "BPlusTreeTest.cpp" "btree.h" "btree.cpp"  "RTreeTest.cpp")
target_link_libraries(
	${TARGET_NAME}
	PRIVATE "shape_lib"
	PRIVATE "hw6_lib"
)