cmake_minimum_required(VERSION 2.8.3)
project(amcl_ros)

find_package(catkin REQUIRED
        COMPONENTS
            roslib
            roscpp
            tf
            dynamic_reconfigure
            rostest
            libamcl
        )

find_package(Boost REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/AMCL.cfg
    cfg/AmclLaser.cfg
)

catkin_package(
    CATKIN_DEPENDS
        roslib
        roscpp
        dynamic_reconfigure
        tf
)

add_executable(amcl
  src/amcl_node.cpp
  src/amcl_wrapper.cpp
  src/amcl_laser_wrapper.cpp
  src/amcl_map_wrapper.cpp
  )

add_dependencies(amcl amcl_gencfg)
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

target_link_libraries(amcl
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install( TARGETS
    amcl
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Bags
download_test_data(http://pr.willowgarage.com/data/amcl/basic_localization_stage_indexed.bag test/basic_localization_stage_indexed.bag 41fe43af189ec71e5e48eb9ed661a655)
download_test_data(http://pr.willowgarage.com/data/amcl/global_localization_stage_indexed.bag test/global_localization_stage_indexed.bag 752f711cf4f6e8d1d660675e2da096b0)
download_test_data(http://pr.willowgarage.com/data/amcl/small_loop_prf_indexed.bag test/small_loop_prf_indexed.bag e4ef0fc006872b43f12ed8a7ce7dcd81)
download_test_data(http://pr.willowgarage.com/data/amcl/small_loop_crazy_driving_prg_indexed.bag test/small_loop_crazy_driving_prg_indexed.bag 4a58d1a7962914009d99000d06e5939c)
download_test_data(http://pr.willowgarage.com/data/amcl/texas_greenroom_loop_indexed.bag test/texas_greenroom_loop_indexed.bag 6e3432115cccdca1247f6c807038e13d)
download_test_data(http://pr.willowgarage.com/data/amcl/texas_willow_hallway_loop_indexed.bag test/texas_willow_hallway_loop_indexed.bag 27deb742fdcd3af44cf446f39f2688a8)
download_test_data(http://pr.willowgarage.com/data/amcl/rosie_localization_stage.bag test/rosie_localization_stage.bag 3347bf3835724cfa45e958c5c1846066)

# Maps
download_test_data(http://pr.willowgarage.com/data/amcl/willow-full.pgm test/willow-full.pgm b84465cdbbfe3e2fb9eb4579e0bcaf0e)
download_test_data(http://pr.willowgarage.com/data/amcl/willow-full-0.05.pgm test/willow-full-0.05.pgm b61694296e08965096c5e78611fd9765)

# Tests
add_rostest(test/set_initial_pose.xml)
add_rostest(test/basic_localization_stage.xml)
add_rostest(test/small_loop_prf.xml)
add_rostest(test/small_loop_crazy_driving_prg.xml)
add_rostest(test/texas_greenroom_loop.xml)
add_rostest(test/rosie_multilaser.xml)
add_rostest(test/texas_willow_hallway_loop.xml)

# Global localization needs some work.
add_rostest(test/global_localization_stage.xml)
