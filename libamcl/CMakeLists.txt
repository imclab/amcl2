cmake_minimum_required(VERSION 2.8.3)
project(libamcl)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES amcl_sensors amcl_map amcl_pf
)

add_library(amcl_pf 
  src/amcl/pf/pf.c 
  src/amcl/pf/pf_kdtree.c 
  src/amcl/pf/pf_pdf.c 
  src/amcl/pf/pf_vector.c
  src/amcl/pf/eig3.c
  src/amcl/pf/pf_draw.c)

add_library(amcl_map
  src/amcl/map/map.c
  src/amcl/map/map_cspace.cpp
  src/amcl/map/map_range.c)

add_library(amcl_sensors
  src/amcl/sensors/amcl_sensor.cpp
  src/amcl/sensors/amcl_odom.cpp
  src/amcl/sensors/amcl_laser.cpp)
target_link_libraries(amcl_sensors amcl_map amcl_pf)


include_directories(include/amcl
  include/amcl/map
  include/amcl/sensors
  include/amcl/pf)

install( TARGETS
  amcl_sensors amcl_map amcl_pf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/amcl/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
