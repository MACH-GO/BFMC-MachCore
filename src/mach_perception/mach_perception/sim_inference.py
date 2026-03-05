import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as ort

import os
from ament_index_python.packages import get_package_share_directory

class YoloSimulationNode(Node):
    def __init__(self):
        super().__init__('yolo_sim_node')
        pkg_share = get_package_share_directory('mach_perception')
        self.model_path = os.path.join(pkg_share, 'model', 'best.onnx')
        
        self.get_logger().info(f"Loading model from: {self.model_path}")

        self.conf_thres = 0.45
        self.iou_thres = 0.45
        self.input_shape = (640, 640)
        self.classes = {
            0: 'HighwayEntry',
            1: 'HighwayExit',
            2: 'Priority',
            3: 'Roundabout',
            4: 'Stop',
            5: 'TrafficSignal',
            6: 'car',
            7: 'crosswalk',
            8: 'noEntry',
            9: 'obstacle-human',
            10: 'obstacle-vehicle',
            11: 'oneway',
            12: 'parking',
            13: 'pedestrian',
            14: 'roadblock'
        } 

        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        try:
            self.session = ort.InferenceSession(self.model_path, providers=providers)
            self.get_logger().info(f"Model loaded on: {self.session.get_providers()[0]}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            return

        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        self.sub = self.create_subscription(Image, '/camera/left/image_raw', self.img_callback, 10)
        self.pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/debug_image', 10) 
        self.bridge = CvBridge()

    def img_callback(self, msg):
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.get_logger().error(f"CV Bridge error: {e}")
                return

            blob = cv2.dnn.blobFromImage(cv_img, 1/255.0, self.input_shape, swapRB=True, crop=False)
            outputs = self.session.run([self.output_name], {self.input_name: blob})
            predictions = np.squeeze(outputs[0]).T

            scores = np.max(predictions[:, 4:], axis=1)
            predictions = predictions[scores > self.conf_thres, :]
            scores = scores[scores > self.conf_thres]
            
            detect_msg = Detection2DArray()
            detect_msg.header = msg.header

            if len(predictions) > 0:
                class_ids = np.argmax(predictions[:, 4:], axis=1)

                boxes = predictions[:, :4]
                input_h, input_w = self.input_shape
                img_h, img_w = cv_img.shape[:2]
                
                x_factor = img_w / input_w
                y_factor = img_h / input_h

                x = boxes[:, 0]
                y = boxes[:, 1]
                w = boxes[:, 2]
                h = boxes[:, 3]

                x1 = (x - w / 2) * x_factor
                y1 = (y - h / 2) * y_factor
                w_scaled = w * x_factor
                h_scaled = h * y_factor
                
                boxes_nms = []
                for i in range(len(x1)):
                    boxes_nms.append([int(x1[i]), int(y1[i]), int(w_scaled[i]), int(h_scaled[i])])

                indices = cv2.dnn.NMSBoxes(boxes_nms, scores.tolist(), self.conf_thres, self.iou_thres)

                if len(indices) > 0:
                    indices = np.array(indices).flatten()

                    for idx in indices:
                        box = boxes_nms[idx]
                        cls_id = int(class_ids[idx])
                        score = float(scores[idx])
                        
                        class_name = self.classes.get(cls_id, str(cls_id))
                        label_text = f"{class_name}: {score:.2f}"

                        detection = Detection2D()
                        detection.bbox.center.position.x = box[0] + box[2] / 2.0
                        detection.bbox.center.position.y = box[1] + box[3] / 2.0
                        detection.bbox.size_x = float(box[2])
                        detection.bbox.size_y = float(box[3])
                        
                        hypothesis = ObjectHypothesisWithPose()
                        hypothesis.hypothesis.class_id = str(cls_id)
                        hypothesis.hypothesis.score = score
                        detection.results.append(hypothesis)
                        detect_msg.detections.append(detection)

                        cv2.rectangle(cv_img, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), (0, 255, 0), 2)
                        cv2.putText(cv_img, label_text, (box[0], box[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            self.pub.publish(detect_msg)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = YoloSimulationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()