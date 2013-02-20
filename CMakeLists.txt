cmake_minimum_required (VERSION 2.6)
project (HuboAchTab)

set(CMAKE_INSTALL_PREFIX /usr )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -O3 -std=c++0x")
set(CMAKE_CXX_FLAGS_PROFILE "-pg" CACHE STRING "Profiling flags")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/grip")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED CONFIG)
find_package(GRIP REQUIRED)

# Search all header and source files
file (GLOB srcs "*.cpp")
file (GLOB hdrs "*.h")

set (wxWidgets_USE_LIBS base core gl)

find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  include_directories(${DARTExt_INCLUDEDIR})
  include_directories(${DART_INCLUDEDIR})
  include_directories(${GRIP_INCLUDEDIR})
  include_directories(${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${DART_LIBRARY_DIRS} ${GRIP_LIBRARY_DIRS} ${WXWIDGETS_SHARED_LIBRARY_DIRS})
  add_executable (HuboAchTab ${srcs} ${hdrs})
  target_link_libraries (HuboAchTab ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES} ${Boost_LIBRARIES})
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
