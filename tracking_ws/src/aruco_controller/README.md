# ARUCO_DETECTOR

OBODROID's CEDT Workshop

Python Implementation of cv2.aruco

parameter can adjust before runtime at aruco_detect_param.yaml 

## Input
- Topic : "/front_camera/image_raw/compressed"
  Type : sensor_msgs/CompressedImage

- Topic : "/front_camera/camera_info"
  Type : sensor_msgs/CameraInfo

## Output
- Topic : "aruco_detector/status"
  Type : std_msgs/Boolean
- TF2 : camera.header.frame_id to "fiducial_<id>"

## Services
- Topic : global "/enable_detections"
  Type : std_srvs/SetBool

