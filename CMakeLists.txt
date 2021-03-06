cmake_minimum_required(VERSION 2.8.3)
project(3dmodel)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)
find_package(PCL 1.2 REQUIRED)
find_package(HDF5)
find_package(FLANN)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 3dmodel
#  CATKIN_DEPENDS cv_bridge image_transport pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS} ${LD_LIBRARY_PATH})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${FLANN_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIR}
)
## Declare a C++ library
# add_library(3dmodel
#   src/${PROJECT_NAME}/3dmodel.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(3dmodel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(3dmodel_node src/3dmodel_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(3dmodel_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(3dmodel_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS 3dmodel 3dmodel_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_3dmodel.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable (build_tree src/build_tree.cpp)
target_link_libraries(build_tree ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})
add_dependencies(build_tree 3dmodel_generate_messages_cpp)
  

#add_executable (nearest_neighbors src/nearest_neighbors.cpp)
#target_link_libraries (nearest_neighbors  ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})
#add_dependencies(nearest_neighbors 3dmodel_generate_messages_cpp)

add_executable (template_alignment src/template_alignment.cpp)
target_link_libraries (template_alignment ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(template_alignment 3dmodel_generate_messages_cpp)

add_executable(VFHestimation src/VFHestimation.cpp)
target_link_libraries(VFHestimation ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(VFHestimation 3dmodel_generate_messages_cpp)

add_executable(FPFHestimation src/FPFHestimation.cpp)
target_link_libraries(FPFHestimation ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(FPFHestimation 3dmodel_generate_messages_cpp)

add_executable(global_hypothesis_verification src/global_hypothesis_verification.cpp)
target_link_libraries(global_hypothesis_verification ${PCL_LIBRARIES})
add_dependencies(global_hypothesis_verification 3dmodel_generate_messages_cpp)

add_executable(recognize src/recognize.cpp)
target_link_libraries(recognize ${PCL_LIBRARIES})
add_dependencies(recognize 3dmodel_generate_messages_cpp)

add_executable(meanshiftCluster src/meanshiftCluster.cpp)
target_link_libraries(meanshiftCluster ${PCL_LIBRARIES})
add_dependencies(meanshiftCluster 3dmodel_generate_messages_cpp)

add_executable(region_growing_segmentation src/region_growing_segmentation.cpp)
target_link_libraries(region_growing_segmentation ${PCL_LIBRARIES})
add_dependencies(region_growing_segmentation 3dmodel_generate_messages_cpp)


add_executable(cylinder_segmentation src/cylinder_segmentation.cpp)
target_link_libraries(cylinder_segmentation ${PCL_LIBRARIES})
add_dependencies(cylinder_segmentation 3dmodel_generate_messages_cpp)

add_executable(cluster_extraction src/cluster_extraction.cpp)
target_link_libraries(cluster_extraction ${PCL_LIBRARIES})
add_dependencies(cluster_extraction 3dmodel_generate_messages_cpp)

add_executable(random_sample_consensus src/random_sample_consensus.cpp)
target_link_libraries (random_sample_consensus ${PCL_LIBRARIES})
add_dependencies(random_sample_consensus 3dmodel_generate_messages_cpp)

add_executable(alignment_prerejective src/alignment_prerejective.cpp)
target_link_libraries (alignment_prerejective ${PCL_LIBRARIES})
add_dependencies(alignment_prerejective 3dmodel_generate_messages_cpp)

add_executable(pairwise_incremental_registration src/pairwise_incremental_registration.cpp)
target_link_libraries (pairwise_incremental_registration ${PCL_LIBRARIES})
add_dependencies(pairwise_incremental_registration 3dmodel_generate_messages_cpp)

add_executable(range_image_visualization src/range_image_visualization.cpp)
target_link_libraries (range_image_visualization ${PCL_LIBRARIES})
add_dependencies(range_image_visualization 3dmodel_generate_messages_cpp)

add_executable(iterative_closest_point src/iterative_closest_point.cpp)
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})
add_dependencies(iterative_closest_point 3dmodel_generate_messages_cpp)

add_executable(correspondence_grouping src/correspondence_grouping.cpp)
target_link_libraries (correspondence_grouping ${PCL_LIBRARIES})
add_dependencies(correspondence_grouping 3dmodel_generate_messages_cpp)

add_executable(range_image_creation src/range_image_creation.cpp)
target_link_libraries (range_image_creation ${PCL_LIBRARIES})
add_dependencies(range_image_creation 3dmodel_generate_messages_cpp)

add_executable(octree_change_detection src/octree_change_detection.cpp)
target_link_libraries (octree_change_detection ${PCL_LIBRARIES})
add_dependencies(octree_change_detection 3dmodel_generate_messages_cpp)

add_executable(octree_search src/octree_search.cpp)
target_link_libraries (octree_search ${PCL_LIBRARIES})
add_dependencies(octree_search 3dmodel_generate_messages_cpp)

add_executable(kdtree_search src/kdtree_search.cpp)
target_link_libraries (kdtree_search ${PCL_LIBRARIES})
add_dependencies(kdtree_search 3dmodel_generate_messages_cpp)

add_executable(narf_keypoint_extraction src/narf_keypoint_extraction.cpp)
target_link_libraries (narf_keypoint_extraction ${PCL_LIBRARIES})
add_dependencies(narf_keypoint_extraction 3dmodel_generate_messages_cpp)

add_executable(openni_viewer_simple src/openni_viewer_simple.cpp)
target_link_libraries(openni_viewer_simple ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(openni_viewer_simple 3dmodel_generate_messages_cpp)

add_executable(concatenate_clouds src/concatenate_clouds.cpp)
target_link_libraries(concatenate_clouds ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(concatenate_clouds 3dmodel_generate_messages_cpp)

add_executable(remove_outliers src/remove_outliers.cpp)
target_link_libraries(remove_outliers ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(remove_outliers 3dmodel_generate_messages_cpp)

add_executable(extract_indices src/extract_indices.cpp)
target_link_libraries(extract_indices ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(extract_indices 3dmodel_generate_messages_cpp)

add_executable(project_inliers src/project_inliers.cpp)
target_link_libraries(project_inliers ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(project_inliers 3dmodel_generate_messages_cpp)

add_executable(statistical_removal src/statistical_removal.cpp)
target_link_libraries(statistical_removal ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(statistical_removal 3dmodel_generate_messages_cpp)

add_executable(passthrough src/passthrough.cpp)
target_link_libraries(passthrough ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(passthrough 3dmodel_generate_messages_cpp)

add_executable(voxelGrid src/voxelGrid.cpp)
target_link_libraries(voxelGrid ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(voxelGrid 3dmodel_generate_messages_cpp)

add_executable(PFHestimation src/PFHestimation.cpp)
target_link_libraries(PFHestimation ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(PFHestimation 3dmodel_generate_messages_cpp)

add_executable(normal_estimation_using_integral_images src/normal_estimation_using_integral_images.cpp)
target_link_libraries(normal_estimation_using_integral_images ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(normal_estimation_using_integral_images 3dmodel_generate_messages_cpp)

add_executable(normalEstimation src/normalEstimation.cpp)
target_link_libraries(normalEstimation ${PCL_LIBRARIES})
add_dependencies(normalEstimation 3dmodel_generate_messages_cpp)

add_executable(transformation src/transformation.cpp)
target_link_libraries(transformation ${PCL_LIBRARIES})
add_dependencies(transformation 3dmodel_generate_messages_cpp)


add_executable(pointCloud_sub src/pointCloud_sub.cpp)
target_link_libraries(pointCloud_sub ${catkin_LIBRARIES})
add_dependencies(pointCloud_sub 3dmodel_generate_messages_cpp)

add_executable(planar_segmentation src/planar_segmentation.cpp)
target_link_libraries(planar_segmentation ${PCL_LIBRARIES})
add_dependencies(planar_segmentation 3dmodel_generate_messages_cpp)

add_executable (cloud_viewer src/cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})
add_dependencies(cloud_viewer 3dmodel_generate_messages_cpp)

add_executable(timeSyncDemo src/timeSyncDemo.cpp)
target_link_libraries(timeSyncDemo ${catkin_LIBRARIES})
add_dependencies(timeSyncDemo 3dmodel_generate_messages_cpp)

add_executable(cloudSegmentation src/cloudSegmentation.cpp)
target_link_libraries(cloudSegmentation ${catkin_LIBRARIES} )
add_dependencies(cloudSegmentation 3dmodel_generate_messages_cpp)







