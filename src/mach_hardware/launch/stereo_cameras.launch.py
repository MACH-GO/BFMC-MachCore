from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Install gscam: sudo apt install ros-humble-gscam

    left_pipeline = (
        "nvarguscamerasrc sensor_id=0 ! "
        "video/x-raw(memory:NVMM),width=2304,height=1296,framerate=30/1,format=NV12 ! "
        "nvvidconv flip-method=2 ! video/x-raw,width=1280,height=720,format=I420 ! "
        "videoconvert"
    )

    right_pipeline = (
        "nvarguscamerasrc sensor_id=1 ! "
        "video/x-raw(memory:NVMM),width=2304,height=1296,framerate=30/1,format=NV12 ! "
        "nvvidconv flip-method=2 ! video/x-raw,width=1280,height=720,format=I420 ! "
        "videoconvert"
    )

    left_cam = Node(
        package="gscam",
        executable="gscam",
        name="left_camera",
        output="screen",
        parameters=[{
            "gscam_config": left_pipeline,
            "frame_id": "left_camera_frame",
            "camera_name": "left",
            # "camera_info_url": [],
        }],
        remappings=[
            ("camera/image_raw", "left/image_raw"),
            ("camera/camera_info", "left/camera_info"),
        ],
    )

    right_cam = Node(
        package="gscam",
        executable="gscam",
        name="right_camera",
        output="screen",
        parameters=[{
            "gscam_config": right_pipeline,
            "frame_id": "right_camera_frame",
            "camera_name": "right",
            # "camera_info_url": [],
        }],
        remappings=[
            ("camera/image_raw", "right/image_raw"),
            ("camera/camera_info", "right/camera_info"),
        ],
    )

    return LaunchDescription([left_cam, right_cam])
