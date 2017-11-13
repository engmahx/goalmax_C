cmake_minimum_required (VERSION 2.8.11)
project (STEPS)
set (CMAKE_CXX_STANDARD 11)

find_package (libmongoc-1.0 1.7 REQUIRED)

#find_library(M_LIB m)


add_executable (goalmaxServer ../goalmaxServer.cpp)

target_link_libraries(goalmaxServer /home/goalmax/goalmax.so)
target_link_libraries(goalmaxServer /usr/local/lib/libpaho-mqtt3c.so.1)

target_include_directories (goalmaxServer PRIVATE "${MONGOC_INCLUDE_DIRS}")
target_link_libraries (goalmaxServer PRIVATE "${MONGOC_LIBRARIES}")
target_compile_definitions (goalmaxServer PRIVATE "${MONGOC_DEFINITIONS}")





# Specify the minimum version you require.
#find_package (libmongoc-1.0 1.7 REQUIRED)

#message ("--   mongoc found version \"${MONGOC_VERSION}\"")
#message ("--   mongoc include path \"${MONGOC_INCLUDE_DIRS}\"")
#message ("--   mongoc libraries \"${MONGOC_LIBRARIES}\"")

# The "hello_mongoc.c" sample program is shared among four tests.
#add_executable (hello_mongoc ../../hello_mongoc.c)
#target_include_directories (hello_mongoc PRIVATE "${MONGOC_INCLUDE_DIRS}")
#target_link_libraries (hello_mongoc PRIVATE "${MONGOC_LIBRARIES}")
#target_compile_definitions (hello_mongoc PRIVATE "${MONGOC_DEFINITIONS}")