cmake_minimum_required(VERSION 2.8)
project(summercamp)

include(cmake/PICMake.cmake)
set(CMAKE_CXX_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(src)

pi_add_target(summercamp SHARED src REQUIRED System OpenCV Ceres)

pi_report_target()

# Now do make install
pi_install(TARGETS summercamp)
install(PROGRAMS scripts/simulate DESTINATION bin)
install(DIRECTORY data DESTINATION .)

