(ros::load-ros-manifest "roseus")
(defun image-collision-detection-init ()
  (ros::wait-for-service "image_collision_detection_init")
  (ros::service-call "image_collision_detection_init" (instance std_srvs::EmptyRequest :init)))

