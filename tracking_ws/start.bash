__byobu() {
    session=$1
    window_no=$2
    name=$3
    cmd=$4
    target_ros_master=$5
    # If No Session
    if ! (byobu list-session | grep -q "$session"); then
        echo " NO SESSION - CREATING NEW ONE"
        byobu new-session -d -s $session
    fi

    byobu new-window -t $session:$window_no
    byobu rename-window -t $session:$window_no "$name"
    byobu select-window -t $session:$window_no
    echo "Apply Command : " $cmd " on window : " $window_no
    byobu send-keys -t $session:$window_no "$cmd" C-m
}

byobu_roslaunch() {
    session=$1
    window_no=$2
    window_name=$3
    cmd=$4
    name=$5
    __byobu $session $window_no $window_name "rosrun rosmon rosmon $cmd --name mon_$name"
}

byobu_bash() {
    session=$1
    window_no=$2
    name=$3
    cmd=$4
    __byobu $1 $2 $3 "$cmd"
}

#########################################################
# MAIN
#########################################################
echo "[STARTING TRACKING_ROBOT]"

byobu_bash CAMERA 0 CAMERA "ros2 launch aruco_controller main_launch.py"
sleep 3s

# 2. Bringup
byobu_bash CAMERA 1 STREAM "ros2 run web_video_server web_video_server"
byobu_bash CORE 0 STATIC_TF "ros2 launch aruco_following_controller static_tf.launch.py"
byobu_bash CORE 1 CONTROLLER "ros2 launch aruco_following_controller controller.launch.py"
byobu_bash CORE 2 VELMUX "ros2 run keyboard_control vel_mux"
byobu_bash CORE 3 KEYBOARD "ros2 run keyboard_control keyboard_control"

sleep 1s