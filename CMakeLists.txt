cmake_minimum_required (VERSION 2.6)
project (HuboAchTab)

###############
# Build configuration

set(CMAKE_INSTALL_PREFIX /usr )
set(CPACK_PACKAGE_NAME "hubo-ach-tab")

set(HACHT_MAJOR_VERSION "0")
set(HACHT_MINOR_VERSION "1")
set(HACHT_PATCH_VERSION "0")
set(PKG_DESC "HACHT is a DART/GRIP application which emulates the HUBO robot using ach.")
set(HACHT_VERSION "${HACHT_MAJOR_VERSION}.${HACHT_MINOR_VERSION}.${HACHT_PATCH_VERSION}")
set(PKG_EXTERNAL_DEPS "hubo-ach, hubo-ach-dev, ach, grip, dart, flann, ccd, fcl")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -O3 -std=c++0x")
set(CMAKE_CXX_FLAGS_PROFILE "-pg" CACHE STRING "Profiling flags")

###############
# libraries and headers

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/grip")

# wxwidgets
set (wxWidgets_USE_LIBS base core gl)
find_package(wxWidgets REQUIRED)

# dart, grip
find_package(DARTExt REQUIRED)
find_package(DART REQUIRED CONFIG)
find_package(GRIP REQUIRED)

# ach, hubo-ach
set(ACH_LIBRARIES "/usr/lib/libach.so")
set(HUBO_LIBRARIES "/usr/lib/libhuboparams.so")

###############
# Actual build process

file (GLOB srcs "*.cpp")
file (GLOB hdrs "*.h")

include (${wxWidgets_USE_FILE})
include_directories(${DARTExt_INCLUDEDIR})
include_directories(${DART_INCLUDEDIR})
include_directories(${GRIP_INCLUDEDIR})
include_directories(${WXWIDGETS_SHARED_INCLUDE_DIRS})
link_directories(${CMAKE_SOURCE_DIR}/lib ${DART_LIBRARY_DIRS} ${GRIP_LIBRARY_DIRS} ${WXWIDGETS_SHARED_LIBRARY_DIRS})

add_executable (HuboAchTab ${srcs} ${hdrs})

target_link_libraries (HuboAchTab ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES} ${Boost_LIBRARIES} ${HUBO_LIBRARIES} ${ACH_LIBRARIES})


###############
# Installation setup

install(TARGETS HuboAchTab RUNTIME DESTINATION bin)
install(DIRECTORY hubo-models DESTINATION share/hubo-ach-tab)

set(CPACK_SYSTEM_NAME "i386")
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  set(CPACK_SYSTEM_NAME "amd64")    
endif()
set(CPACK_PACKAGE_VERSION_MAJOR ${HACHT_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${HACHT_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${HACHT_PATCH_VERSION})
set(CPACK_DEBIAN_PACKAGE_DEPENDS "dart (>=2.3.0), libwxgtk2.8-dev, libach-dev, hubo-ach-dev")

set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Golems (http://github.com/golems/)")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PKG_DESC}")
set(CPACK_PACKAGE_VENDOR "Humanoids Robotics Lab at GT")

include(CPack)

