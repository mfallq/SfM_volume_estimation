CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)
PROJECT(pic_stuff CXX) # defines ${CMAKE_PROJECT_NAME}(top level) PROJECT_NAME(most recent)
find_package(mlib REQUIRED)
include(${mlib_CMAKE_HELPERS})
message("\n${line}\n")
include_directories( ${mlib_INCLUDE_DIRS})
set(LIBS ${mlib_LIBRARIES})

set(languageused " -std=c++11 ")
add_definitions(${languageused})

#build config
BuildConfig()
# Warning behaviour
WarningConfig()
# opt flags per compiler
OptimizationConfig()

#find_package_OpenCV()
find_package( OpenCV 3.0.0 EXACT REQUIRED )
list(APPEND LIBS ${OPENCV_LIBRARIES})
find_package(gflags REQUIRED)
list(APPEND LIBS gflags)

option(withcuda "WITH_CUDA" ON)
if (withcuda)
    add_definitions(-DWITH_CUDA)
endif()

add_definitions(-DWITH_IDS)
find_library(UEYE_LIBRARY ueye_api)
include_directories(${UEYE_INCLUDE_DIR})

list(APPEND LIBS ${UEYE_LIBRARY})

#file(GLOB_RECURSE files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.h" "*.cpp")
file(GLOB_RECURSE LIB_SOURCES "./lib" "*.cpp")
message("LIB_SOURCES: ${LIB_SOURCES}")
file(GLOB_RECURSE LIB_HEADERS "./lib" "*.h")
add_library(mylib ${LIB_SOURCES} ${LIB_HEADERS})
target_link_libraries(mylib ${LIBS})
#list(APPEND LIBS mylib)
set(LIBS ${LIBS} mylib)
#add_executable(main main.cpp ${files})
#add_executable(lines lines.cpp ${files})
add_executable(points main.cpp)
target_link_libraries(points ${LIBS})
add_executable(lines lines.cpp)
target_link_libraries(lines ${LIBS})


