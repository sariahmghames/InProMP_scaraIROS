rostopic list:

/clicked_point
/diagnostics
/graspberry_camera1/aligned_depth_to_color/camera_info
/graspberry_camera1/aligned_depth_to_color/image_raw
/graspberry_camera1/color/camera_info
/graspberry_camera1/color/image_raw
/graspberry_camera1/depth/camera_info
/graspberry_camera1/depth/color/points
/graspberry_camera1/depth/image_rect_raw
/graspberry_camera1/extrinsics/depth_to_color
/graspberry_camera1/realsense2_camera_manager/bond
/graspberry_camera1/rgb_camera/parameter_descriptions
/graspberry_camera1/rgb_camera/parameter_updates
/graspberry_camera1/stereo_module/parameter_descriptions
/graspberry_camera1/stereo_module/parameter_updates
/initialpose
/move_base_simple/goal
/rosout
/rosout_agg
/tf
/tf_static



run :
roslaunch graspberry_perception camera_publisher.launch

roslaunch graspberry_perception 2D_detector.launch 
