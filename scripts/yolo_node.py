#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from kinova_msgs.msg import FingerPosition  # Ensure this is the correct message type for finger positions
from std_msgs.msg import String  # Assuming head_position topic uses String messages
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO

class YoloRosNode:
    def __init__(self):
        rospy.init_node('yolo_ros_node', anonymous=True)
        self.bridge = CvBridge()
        self.model = YOLO('/home/tnlab/catkin_ws/src/yolo-ros/best.pt')

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.finger_position_sub = rospy.Subscriber("/j2n6s300_driver/out/finger_position", FingerPosition, self.finger_position_callback)
        self.head_position_sub = rospy.Subscriber("/head_position", String, self.head_position_callback)
        self.image_pub = rospy.Publisher("/yolo/image_with_bboxes", Image, queue_size=10)
        self.mask_2d_pub = rospy.Publisher("/yolo/segmentation_mask_2d", Image, queue_size=10)
        self.mask_2d_visual_pub = rospy.Publisher("/yolo/segmentation_mask_2d_visual", Image, queue_size=10)
        self.image_with_dots_pub = rospy.Publisher("/yolo/image_with_center_dots", Image, queue_size=10)
        self.automove_pub = rospy.Publisher("/yolo/automove", String, queue_size=10)

        self.finger_1_position = 0
        self.head_position = "neutral"

        # Define a color map for different object classes
        self.color_map = {
            1: (200, 200, 200),    # White for workspace
            2: (0, 100, 0),      # Green for jaco
            3: (0, 0, 100),      # Blue for object
            4: (100, 0, 0)       # Red for bin
        }

        # Define a name-to-ID mapping
        self.name_to_id = {
            'workspace': 1,
            'jaco': 2,
            'object': 3,
            'bin': 4
        }

        # Flags to ensure the action is published only once
        # self.grasp_published = False
        # self.release_published = False

    def finger_position_callback(self, data):
        self.finger_1_position = data.finger1  # Adjust this according to the actual structure of FingerPosition message

    def head_position_callback(self, data):
        self.head_position = data.data  # Assuming the head_position topic sends String messages

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        results = self.model(cv_image)
        annotated_frame = results[0].plot()  # Automatically draw bounding boxes and masks

        # Create a copy of the original image to draw dots on
        image_with_dots = cv_image.copy()

        gripper_pos_x, gripper_pos_y = None, None

        # # Finding the gripper position
        # for i, box in enumerate(results[0].boxes.xyxy):
        #     class_name = results[0].names[int(results[0].boxes.cls[i])]
        #     if class_name == 'jaco':
        #         x1, y1, x2, y2 = map(int, box)
        #         gripper_pos_x = x1 + 60
        #         gripper_pos_y = y1

        # Drawing center dots for each detected object
        for i, box in enumerate(results[0].boxes.xyxy):
            class_name = results[0].names[int(results[0].boxes.cls[i])]
            x1, y1, x2, y2 = map(int, box)

            if class_name == "workspace":
                continue
            
            # Calculate center of the bounding box
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            if class_name == 'jaco':
                gripper_pos_x = x1 + 60
                gripper_pos_y = y1

                center_x, center_y = x1+55 , y1+50

            
            # Draw a dot at the center (red color, filled circle)
            cv2.circle(image_with_dots, (center_x, center_y), 5, (0, 0, 255), -1)


        masks = results[0].masks
        if masks is not None and masks.data is not None:
            mask_data = masks.data.cpu().numpy()

            height, width = cv_image.shape[:2]

            combined_mask_2d = np.zeros((height, width), dtype=np.uint8)

            for i, mask in enumerate(mask_data):
                class_name = results[0].names[int(results[0].boxes.cls[i])]
                class_id = self.name_to_id.get(class_name, 0)

                if self.finger_1_position < 1000 and class_name == 'bin':
                    class_id = 1  # Treat "bin" as background

                if class_name == 'object' or class_name == 'bin':
                    obj_pos_x = (results[0].boxes.xyxy[i][0] + results[0].boxes.xyxy[i][2]) / 2
                    obj_pos_y = (results[0].boxes.xyxy[i][1] + results[0].boxes.xyxy[i][3]) / 2
                    

                    if self.head_position == "Left" and gripper_pos_x < obj_pos_x + 30:
                        class_id = 1  # Treat as part of the workspace
                    elif self.head_position == "Right" and gripper_pos_x > obj_pos_x:
                        class_id = 1  # Treat as part of the workspace
                    elif self.head_position == "Neutral" and (abs(gripper_pos_x - obj_pos_x) > 40):
                        class_id = 1  # Treat as part of the workspace

                    # Position info for auto grasp and release

                    grasp_executed = rospy.get_param('/grasp_executed', False)
                    release_executed = rospy.get_param('/release_executed', False)

                    # if self.head_position == "Neutral" and class_name == "object" and class_id != 1 and (abs(gripper_pos_y - obj_pos_y) < 25):
                    #     if not grasp_executed:
                    #         self.automove_pub.publish("Grasp")
                            
                    # elif self.head_position == "Neutral" and class_name == "bin" and class_id != 1 and (abs(gripper_pos_y - obj_pos_y) < 30):
                    #     if not release_executed:
                    #         self.automove_pub.publish("Release")
                            
                    
                    # if self.head_position == "Neutral" and class_name == "object" and class_id != 1 and (abs(gripper_pos_y - obj_pos_y) < 8):
                    #     if not grasp_executed:
                    #         self.automove_pub.publish("Grasp")


                    if class_name == "object" and (abs(gripper_pos_y - obj_pos_y) < 8): # ICRA 
                        if not grasp_executed:
                            self.automove_pub.publish("Grasp")
                            
                    # elif class_name == "bin" and class_id != 1 and (abs(gripper_pos_y - obj_pos_y) < 11):
                    #     if not release_executed:
                    #         self.automove_pub.publish("Release")
                            

                color = self.color_map.get(class_id, (255, 255, 255))

                resized_mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
                combined_mask_2d[resized_mask > 0] = class_id

            combined_mask_2d_visual = np.zeros((height, width, 3), dtype=np.uint8)
            for class_id, color in self.color_map.items():
                combined_mask_2d_visual[combined_mask_2d == class_id] = color

            try:
                # Publish the images
                ros_image_with_dots = self.bridge.cv2_to_imgmsg(image_with_dots, "bgr8")
                self.image_with_dots_pub.publish(ros_image_with_dots)

                ros_mask_2d = self.bridge.cv2_to_imgmsg(combined_mask_2d, "mono8")
                self.mask_2d_pub.publish(ros_mask_2d)

                ros_mask_2d_visual = self.bridge.cv2_to_imgmsg(combined_mask_2d_visual, "bgr8")
                self.mask_2d_visual_pub.publish(ros_mask_2d_visual)
                ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                self.image_pub.publish(ros_image)

            except CvBridgeError as e:
                rospy.logerr(e)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = YoloRosNode()
    node.run()
