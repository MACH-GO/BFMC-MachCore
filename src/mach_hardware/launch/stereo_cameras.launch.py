import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("mach_hardware")
    
    left_yaml = os.path.join(
    	pkg_share,
    	"config",
    	"left.yaml"
    )
    
    right_yaml = os.path.join(
    	pkg_share,
    	"config",
    	"right.yaml"
    )

    left_pipeline = (
        "nvarguscamerasrc wbmode=0 sensor_id=1 ! "
        "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12 ! "
        "nvvidconv flip-method=2 ! video/x-raw,width=960,height=720,format=I420 ! "
        "videoconvert"
    )

    right_pipeline = (
        "nvarguscamerasrc wbmode=0 sensor_id=0 ! "
        "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12 ! "
        "nvvidconv flip-method=2 ! video/x-raw,width=960,height=720,format=I420 ! "
        "videoconvert"
    )

    left_cam = Node(
        package="gscam",
        executable="gscam_node",
        name="left_camera",
        output="screen",
        parameters=[{
            "gscam_config": left_pipeline,
            "frame_id": "left_camera_frame",
            "camera_name": "left_camera",
            "camera_info_url": "file://" + left_yaml,
        }],
        remappings=[
            ("camera/image_raw", "camera/left/image_raw"),
            ("camera/camera_info", "camera/left/camera_info"),
        ],
    )

    right_cam = Node(
        package="gscam",
        executable="gscam_node",
        name="right_camera",
        output="screen",
        parameters=[{
            "gscam_config": right_pipeline,
            "frame_id": "right_camera_frame",
            "camera_name": "right_camera",
            "camera_info_url": "file://" + right_yaml,
        }],
        remappings=[
            ("camera/image_raw", "camera/right/image_raw"),
            ("camera/camera_info", "camera/right/camera_info"),
        ],
    )

    return LaunchDescription([left_cam, right_cam])