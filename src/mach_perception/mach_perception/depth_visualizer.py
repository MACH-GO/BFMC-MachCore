import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
import cv2

class DepthVisualizer(Node):
    def __init__(self):
        super().__init__('depth_visualizer')
        
        self.sub = self.create_subscription(
            DisparityImage,
            '/camera/disparity',
            self.callback,
            10)
        
        self.pub = self.create_publisher(Image, '/perception/depth_heatmap', 10)
        self.bridge = CvBridge()

    def callback(self, msg):
        try:
            disparity = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='32FC1')
            
            mask = disparity > 0

            f = msg.f
            T = msg.t
            
            h, w = disparity.shape
            center_disp = disparity[h//2, w//2]
            
            distance_text = "Range: Inf"
            if center_disp > 0:
                depth = (f * T) / center_disp
                distance_text = f"Range: {depth:.2f}m"

            norm_disp = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            heatmap = cv2.applyColorMap(norm_disp, cv2.COLORMAP_JET)
            heatmap[~mask] = [0, 0, 0]

            cv2.putText(heatmap, distance_text, (50, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            cv2.drawMarker(heatmap, (w//2, h//2), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

            out_msg = self.bridge.cv2_to_imgmsg(heatmap, encoding='bgr8')
            out_msg.header = msg.header
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()