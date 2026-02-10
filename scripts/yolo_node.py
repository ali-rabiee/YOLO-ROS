#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from kinova_msgs.msg import FingerPosition  # Ensure this is the correct message type for finger positions
from std_msgs.msg import String  # Assuming head_position topic uses String messages
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO
import json

from yolo_model_loader import resolve_model_path, load_ultralytics_yolo

class YoloRosNode:
    def __init__(self):
        # NOTE: do NOT use anonymous=True here; it breaks private params (~grid_crop_*)
        # passed from roslaunch because the node name gets a random suffix.
        rospy.init_node('yolo_node', anonymous=False)
        self.bridge = CvBridge()

        # -----------------------
        # Model selection (YOLO26 vs YOLOv8)
        # -----------------------
        # Switch between models via params:
        #   rosparam set /yolo_node/model_variant yolo26|yolov8
        #   rosparam set /yolo_node/model_path /abs/path/to/weights.pt   (optional override)
        #
        # Default is YOLO26.
        self.model_variant = str(rospy.get_param("~model_variant", "yolo26")).strip().lower()
        self.model_path = rospy.get_param("~model_path", None)
        self.fallback_to_yolov8_on_error = bool(
            rospy.get_param("~fallback_to_yolov8_on_error", True)
        )

        resolved_path = None
        try:
            resolved_path = resolve_model_path(self.model_variant, self.model_path)
            self.model = load_ultralytics_yolo(resolved_path)
            rospy.loginfo(f"Loaded model_variant='{self.model_variant}' from '{resolved_path}'")
        except Exception as e:
            if self.fallback_to_yolov8_on_error and self.model_variant != "yolov8":
                rospy.logwarn(
                    f"Failed loading model_variant='{self.model_variant}' (error: {e}). "
                    "Falling back to 'yolov8'."
                )
                self.model_variant = "yolov8"
                resolved_path = resolve_model_path(self.model_variant, None)
                self.model = load_ultralytics_yolo(resolved_path)
                rospy.loginfo(f"Loaded fallback model_variant='yolov8' from '{resolved_path}'")
            else:
                rospy.logerr(f"Failed to load YOLO model: {e}")
                raise

        # Grid ROI cropping inside the detected workspace bbox.
        # Example: crop_top_ratio=0.5 means "use bottom half of the workspace" for the 3x3 grid.
        self.grid_crop_top_ratio = float(rospy.get_param("~grid_crop_top_ratio", 0.25))
        self.grid_crop_bottom_ratio = float(rospy.get_param("~grid_crop_bottom_ratio", 0.0))
        self._last_param_refresh = rospy.Time(0)

        # -----------------------
        # Segmentation masks
        # -----------------------
        # YOLO segmentation models output both boxes + masks. Publishing masks can be controlled via:
        #   ~publish_segmentation_masks: bool (default True)
        #   ~mask_publish_mode: "always" | "on_key"
        #   ~mask_publish_key: string (default: "mask") used when mode="on_key"
        #
        # "on_key" keeps backward-compat behavior: only publish when keyboard_state == key (callback2),
        # or head_position == key (callback1).
        self.publish_segmentation_masks = bool(rospy.get_param("~publish_segmentation_masks", True))
        self.mask_publish_mode = str(rospy.get_param("~mask_publish_mode", "always")).strip().lower()
        self.mask_publish_key = str(rospy.get_param("~mask_publish_key", "mask")).strip().lower()

        # -----------------------
        # Mask-based yaw estimation (purely image-based)
        # -----------------------
        # If enabled and masks are available, compute an in-image yaw for each detected instance
        # from the principal axis of its segmentation mask (PCA on mask pixels) and attach it
        # into /yolo/detections_json as:
        #   det["mask_center_xy"] = [cx, cy]
        #   det["mask_yaw_rad"]   = yaw  (radians, image coords; +x right, +y down)
        #   det["mask_yaw_ratio"] = major/minor axis ratio (bigger = more elongated)
        #
        # Downstream (PRIME) can map this yaw into robot/world yaw with simple flips/offset.
        self.compute_mask_yaw = bool(rospy.get_param("~compute_mask_yaw", True))
        self.mask_yaw_min_pixels = int(rospy.get_param("~mask_yaw_min_pixels", 80))
        self.mask_yaw_min_ratio = float(rospy.get_param("~mask_yaw_min_ratio", 1.25))
        # Debug overlay: draw the mask principal axis on /yolo/image_with_grid
        self.draw_mask_yaw_overlay = bool(rospy.get_param("~draw_mask_yaw_overlay", True))
        self.mask_yaw_overlay_len_px = int(rospy.get_param("~mask_yaw_overlay_len_px", 60))

        # -----------------------
        # Image input topic
        # -----------------------
        # If you see cv_bridge errors like:
        #   "[16UC1] is not a color format ... [bgr8]"
        # then you're feeding a depth topic into this node. Set:
        #   ~image_topic:=/camera/color/image_raw
        # (or whatever your RGB topic is).
        self.image_topic = str(rospy.get_param("~image_topic", "/camera/color/image_raw")).strip()
        self.allow_non_rgb_input = bool(rospy.get_param("~allow_non_rgb_input", False))
        self._last_bad_encoding_log = rospy.Time(0)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback2)
        self.finger_position_sub = rospy.Subscriber("/j2n6s300_driver/out/finger_position", FingerPosition, self.finger_position_callback)
        self.head_position_sub = rospy.Subscriber("/head_position", String, self.head_position_callback)
        self.keyboard_state_sub = rospy.Subscriber("/keyboard_state", String, self.keyboard_state_callback)
        self.image_pub = rospy.Publisher("/yolo/image_with_bboxes", Image, queue_size=10)
        # New: publish detections in a structured (JSON) format for PRIME
        self.detections_pub = rospy.Publisher("/yolo/detections_json", String, queue_size=10)
        # New: publish an image with grid overlay + per-object cell labels (A1..C3)
        self.image_with_grid_pub = rospy.Publisher("/yolo/image_with_grid", Image, queue_size=10)
        if self.publish_segmentation_masks:
            self.mask_2d_pub = rospy.Publisher("/yolo/segmentation_mask_2d", Image, queue_size=10)
            self.mask_2d_visual_pub = rospy.Publisher("/yolo/segmentation_mask_2d_visual", Image, queue_size=10)
        else:
            self.mask_2d_pub = None
            self.mask_2d_visual_pub = None
        self.image_with_dots_pub = rospy.Publisher("/yolo/image_with_center_dots", Image, queue_size=10)
        self.automove_pub = rospy.Publisher("/yolo/automove", String, queue_size=10)

        self.finger_1_position = 0
        self.head_position = "Neutral"
        self.keyboard_state = "Neutral"

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

    def _imgmsg_to_bgr(self, msg):
        """
        Convert a sensor_msgs/Image into a BGR OpenCV image for inference.
        Returns None if the message is a depth/non-RGB image (unless allow_non_rgb_input is true).
        """
        enc = str(getattr(msg, "encoding", "")).lower().strip()

        # Common RGB encodings
        rgb_like = {"bgr8", "rgb8", "bgra8", "rgba8"}
        gray_like = {"mono8", "8uc1"}
        depth_like = {"mono16", "16uc1", "32fc1"}

        try:
            if enc in rgb_like:
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if enc in gray_like:
                gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            if enc in depth_like:
                if not self.allow_non_rgb_input:
                    now = rospy.Time.now()
                    if (now - self._last_bad_encoding_log).to_sec() > 2.0:
                        self._last_bad_encoding_log = now
                        rospy.logerr(
                            f"Received a depth/non-RGB image on '{self.image_topic}' (encoding='{enc}'). "
                            "Set ~image_topic to your RGB topic (e.g. /camera/color/image_raw)."
                        )
                    return None
                # Optional: visualize depth as 8-bit BGR so the pipeline can keep running.
                cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                cv_depth = np.nan_to_num(cv_depth).astype(np.float32)
                # Normalize for visualization (not physically meaningful for inference).
                dmin, dmax = float(np.min(cv_depth)), float(np.max(cv_depth))
                if dmax <= dmin:
                    return None
                depth_8u = ((cv_depth - dmin) * (255.0 / (dmax - dmin))).clip(0, 255).astype(np.uint8)
                depth_color = cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)
                return depth_color

            # Unknown encoding
            if not self.allow_non_rgb_input:
                now = rospy.Time.now()
                if (now - self._last_bad_encoding_log).to_sec() > 2.0:
                    self._last_bad_encoding_log = now
                    rospy.logerr(
                        f"Unsupported image encoding '{enc}' on '{self.image_topic}'. "
                        "Set ~image_topic to an RGB Image topic."
                    )
                return None
            # Best-effort passthrough
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if cv_img is None:
                return None
            if len(cv_img.shape) == 2:
                return cv2.cvtColor(cv_img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
            return cv_img
        except CvBridgeError as e:
            now = rospy.Time.now()
            if (now - self._last_bad_encoding_log).to_sec() > 2.0:
                self._last_bad_encoding_log = now
                rospy.logerr(f"cv_bridge exception on '{self.image_topic}': {e}")
            return None

    def _refresh_grid_crop_params(self):
        """Re-read crop params periodically so tuning shows up live."""
        now = rospy.Time.now()
        if (now - self._last_param_refresh).to_sec() < 0.5:
            return
        self._last_param_refresh = now

        top = float(rospy.get_param("~grid_crop_top_ratio", self.grid_crop_top_ratio))
        bottom = float(rospy.get_param("~grid_crop_bottom_ratio", self.grid_crop_bottom_ratio))
        # clamp
        top = max(0.0, min(0.95, top))
        bottom = max(0.0, min(0.95, bottom))
        if top + bottom >= 0.95:
            top, bottom = 0.5, 0.0

        if abs(top - self.grid_crop_top_ratio) > 1e-6 or abs(bottom - self.grid_crop_bottom_ratio) > 1e-6:
            rospy.loginfo(f"Updated grid crop params: top={top:.2f} bottom={bottom:.2f}")
            self.grid_crop_top_ratio = top
            self.grid_crop_bottom_ratio = bottom

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def _as_float(v):
        """Convert torch/np scalars to python float; return None on failure."""
        if v is None:
            return None
        try:
            # torch scalar / numpy scalar
            if hasattr(v, "item"):
                return float(v.item())
        except Exception:
            pass
        try:
            return float(v)
        except Exception:
            return None

    @staticmethod
    def _cell_label_from_pixel(cx, cy, ws_bbox_xyxy):
        """
        Map a pixel center (cx, cy) to PRIME-style grid label A1..C3 based on the workspace bbox.
        ws_bbox_xyxy: [x1,y1,x2,y2] in pixels.
        """
        if ws_bbox_xyxy is None:
            return None, None, None
        x1, y1, x2, y2 = ws_bbox_xyxy
        # If outside bbox, it's not on the workspace/grid.
        try:
            fx = float(cx)
            fy = float(cy)
        except Exception:
            return None, None, None
        if fx < float(x1) or fx > float(x2) or fy < float(y1) or fy > float(y2):
            return None, None, None
        w = max(1.0, float(x2 - x1))
        h = max(1.0, float(y2 - y1))
        col = int((float(cx) - x1) / (w / 3.0))
        row = int((float(cy) - y1) / (h / 3.0))
        col = YoloRosNode._clamp(col, 0, 2)
        row = YoloRosNode._clamp(row, 0, 2)
        row_letter = ["A", "B", "C"][row]
        label = f"{row_letter}{col+1}"
        cell_index = row * 3 + col
        return label, row, col

    @staticmethod
    def _crop_bbox_vertical(bbox_xyxy, crop_top_ratio, crop_bottom_ratio):
        """
        Crop a bbox vertically while keeping left/right the same.
        crop_top_ratio: fraction of height removed from the top (0..0.95)
        crop_bottom_ratio: fraction of height removed from the bottom (0..0.95)
        """
        if bbox_xyxy is None:
            return None
        x1, y1, x2, y2 = bbox_xyxy
        h = max(1, y2 - y1)
        ct = max(0.0, min(0.95, float(crop_top_ratio)))
        cb = max(0.0, min(0.95, float(crop_bottom_ratio)))
        # Ensure we don't invert
        if ct + cb >= 0.95:
            ct = 0.5
            cb = 0.0
        new_y1 = int(round(y1 + ct * h))
        new_y2 = int(round(y2 - cb * h))
        if new_y2 <= new_y1:
            return [x1, y1, x2, y2]
        return [int(x1), int(new_y1), int(x2), int(new_y2)]

    def finger_position_callback(self, data):
        self.finger_1_position = data.finger1  # Adjust this according to the actual structure of FingerPosition message

    def head_position_callback(self, data):
        self.head_position = data.data  # Assuming the head_position topic sends String messages

    def keyboard_state_callback(self, data):
        self.keyboard_state = data.data


    ############## callback1 works with IMU and callbacck2 works with keyboard #####################

    def image_callback1(self, data):
        cv_image = self._imgmsg_to_bgr(data)
        if cv_image is None:
            return

        results = self.model(cv_image, verbose=False)
        # Avoid drawing segmentation masks on the debug image if supported by this ultralytics version.
        try:
            annotated_frame = results[0].plot(boxes=True, masks=False)
        except TypeError:
            annotated_frame = results[0].plot()

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


        want_masks = self.publish_segmentation_masks
        if want_masks and self.mask_publish_mode == "on_key":
            want_masks = (str(self.head_position).strip().lower() == self.mask_publish_key)
        masks = results[0].masks if want_masks else None
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
                    obj_pos_x = self._as_float((results[0].boxes.xyxy[i][0] + results[0].boxes.xyxy[i][2]) / 2.0)
                    obj_pos_y = self._as_float((results[0].boxes.xyxy[i][1] + results[0].boxes.xyxy[i][3]) / 2.0)
                    
                    if gripper_pos_x is not None and obj_pos_x is not None:
                        if self.head_position == "Left" and gripper_pos_x < obj_pos_x:
                            class_id = 1  # Treat as part of the workspace
                        elif self.head_position == "Right" and gripper_pos_x > obj_pos_x:
                            class_id = 1  # Treat as part of the workspace
                        elif (
                            self.head_position == "Neutral"
                            and class_name == "object"
                            and (abs(gripper_pos_x - obj_pos_x) > 40)
                        ):
                            class_id = 1  # Treat as part of the workspace

                    # Position info for auto grasp and release
                    grasp_executed = rospy.get_param('/grasp_executed', False)
                    release_executed = rospy.get_param('/release_executed', False)

                    if (
                        self.head_position == "Neutral"
                        and class_name == "object"
                        and class_id != 1
                        and gripper_pos_y is not None
                        and obj_pos_y is not None
                        and (abs(gripper_pos_y - obj_pos_y) < 8)
                    ):
                        if not grasp_executed:
                            self.automove_pub.publish("Grasp")
                            
                    elif (
                        self.head_position == "Neutral"
                        and class_name == "bin"
                        and class_id != 1
                        and gripper_pos_y is not None
                        and obj_pos_y is not None
                        and (abs(gripper_pos_y - obj_pos_y) < 2)
                    ):
                        if not release_executed:
                            self.automove_pub.publish("Release")
                            

                            
                    # elif class_name == "bin" and class_id != 1 and (abs(gripper_pos_y - obj_pos_y) < 11):
                    #     if not release_executed:
                    #         self.automove_pub.publish("Release")
                            
                    # if class_name == "object" and (abs(gripper_pos_y - obj_pos_y) < 8): # ICRA 
                    #     if not grasp_executed:
                    #         self.automove_pub.publish("Grasp")

                color = self.color_map.get(class_id, (255, 255, 255))

                resized_mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
                combined_mask_2d[resized_mask > 0] = class_id

            combined_mask_2d_visual = np.zeros((height, width, 3), dtype=np.uint8)
            for class_id, color in self.color_map.items():
                combined_mask_2d_visual[combined_mask_2d == class_id] = color

            try:
                # Publish the images (always)
                ros_image_with_dots = self.bridge.cv2_to_imgmsg(image_with_dots, "bgr8")
                self.image_with_dots_pub.publish(ros_image_with_dots)
                ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                self.image_pub.publish(ros_image)

                # Publish masks only when explicitly enabled
                if self.mask_2d_pub is not None:
                    ros_mask_2d = self.bridge.cv2_to_imgmsg(combined_mask_2d, "mono8")
                    self.mask_2d_pub.publish(ros_mask_2d)
                if self.mask_2d_visual_pub is not None:
                    ros_mask_2d_visual = self.bridge.cv2_to_imgmsg(combined_mask_2d_visual, "bgr8")
                    self.mask_2d_visual_pub.publish(ros_mask_2d_visual)

            except CvBridgeError as e:
                rospy.logerr(e)

        else:
            # Masks disabled/unavailable: still publish the debug images.
            try:
                ros_image_with_dots = self.bridge.cv2_to_imgmsg(image_with_dots, "bgr8")
                self.image_with_dots_pub.publish(ros_image_with_dots)
                ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                self.image_pub.publish(ros_image)
            except CvBridgeError as e:
                rospy.logerr(e)


    ##########################################################
    def image_callback2(self, data):
        cv_image = self._imgmsg_to_bgr(data)
        if cv_image is None:
            return

        # Allow tuning of grid crop params without restarting the node.
        self._refresh_grid_crop_params()

        results = self.model(cv_image, verbose=False)
        # Avoid drawing segmentation masks on the debug image if supported by this ultralytics version.
        try:
            annotated_frame = results[0].plot(boxes=True, masks=False)
        except TypeError:
            annotated_frame = results[0].plot()

        # Create a copy of the original image to draw dots on
        image_with_dots = cv_image.copy()
        image_with_grid = annotated_frame.copy()

        gripper_pos_x, gripper_pos_y = None, None

        # Build structured detections and identify workspace bbox
        detections = []
        workspace_bbox = None
        workspace_score = -1.0
        grid_bbox = None

        try:
            boxes_xyxy = results[0].boxes.xyxy
            boxes_cls = results[0].boxes.cls
            boxes_conf = results[0].boxes.conf
            names = results[0].names
        except Exception:
            boxes_xyxy = []
            boxes_cls = []
            boxes_conf = []
            names = {}

        for i, box in enumerate(boxes_xyxy):
            class_name = names[int(boxes_cls[i])]
            x1, y1, x2, y2 = map(int, box)
            conf = float(boxes_conf[i]) if boxes_conf is not None else 0.0
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # Pixel that should represent the object's contact point on the table.
            # For top-down cameras, bbox bottom-center is usually much more stable for (x,y)
            # mapping than bbox center (parallax from object height).
            pick_x, pick_y = cx, cy
            if class_name in ("object", "bin"):
                pick_x, pick_y = cx, int(y2)

            det = {
                "class": class_name,
                "conf": conf,
                "bbox_xyxy": [x1, y1, x2, y2],
                "center_xy": [cx, cy],
                "pick_xy": [int(pick_x), int(pick_y)],
            }
            detections.append(det)

            if class_name == "workspace":
                # Choose workspace by largest area; tie-break by confidence
                area = float(max(0, x2 - x1) * max(0, y2 - y1))
                score = area + conf * 1e-3
                if score > workspace_score:
                    workspace_score = score
                    workspace_bbox = [x1, y1, x2, y2]

        # Define the effective grid rectangle inside the workspace (same left/right, cropped vertically)
        grid_bbox = self._crop_bbox_vertical(
            workspace_bbox, self.grid_crop_top_ratio, self.grid_crop_bottom_ratio
        )

        # Draw dots for each detected object (use pick point, not bbox center)
        for det in detections:
            class_name = det["class"]
            x1, y1, x2, y2 = det["bbox_xyxy"]

            if class_name == "workspace":
                continue
            
            # Default dot position uses pick point
            center_x, center_y = det.get("pick_xy", det["center_xy"])

            if class_name == 'jaco':
                gripper_pos_x = x1 + 60
                gripper_pos_y = y1

                # Heuristic: gripper tip region inside bbox
                center_x, center_y = x1 + 55, y1 + 50
                det["pick_xy"] = [int(center_x), int(center_y)]

            
            # Draw a dot at the center (red color, filled circle)
            cv2.circle(image_with_dots, (center_x, center_y), 5, (0, 0, 255), -1)

        # Draw workspace bbox + 3x3 grid overlay on image_with_grid
        if workspace_bbox is not None:
            wx1, wy1, wx2, wy2 = workspace_bbox
            # Workspace bbox in yellow
            cv2.rectangle(image_with_grid, (wx1, wy1), (wx2, wy2), (0, 255, 255), 2)

        if grid_bbox is not None:
            gx1, gy1, gx2, gy2 = grid_bbox
            # Grid bbox in magenta
            cv2.rectangle(image_with_grid, (gx1, gy1), (gx2, gy2), (255, 0, 255), 2)
            w = max(1, gx2 - gx1)
            h = max(1, gy2 - gy1)
            # Grid lines
            for k in [1, 2]:
                xk = gx1 + int(k * w / 3.0)
                yk = gy1 + int(k * h / 3.0)
                cv2.line(image_with_grid, (xk, gy1), (xk, gy2), (255, 0, 255), 1)
                cv2.line(image_with_grid, (gx1, yk), (gx2, yk), (255, 0, 255), 1)
            # Label cell centers A1..C3
            for r, row_letter in enumerate(["A", "B", "C"]):
                for c in [1, 2, 3]:
                    cx = gx1 + int((c - 0.5) * w / 3.0)
                    cy = gy1 + int((r + 0.5) * h / 3.0)
                    cv2.putText(
                        image_with_grid,
                        f"{row_letter}{c}",
                        (cx - 15, cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2,
                    )

        # Add per-detection cell labels on image_with_grid + also include in JSON.
        # Use pick point so the visual label matches PRIME's discretization.
        for det in detections:
            if det["class"] == "workspace":
                continue
            cx, cy = det.get("pick_xy", det["center_xy"])
            cell_label, row, col = self._cell_label_from_pixel(cx, cy, grid_bbox)
            det["grid_cell"] = cell_label
            det["grid_row"] = row
            det["grid_col"] = col
            if cell_label is not None:
                cv2.putText(
                    image_with_grid,
                    cell_label,
                    (cx + 5, cy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                )

        # Optional segmentation mask publishing (disabled by default)
        want_masks = self.publish_segmentation_masks
        if want_masks and self.mask_publish_mode == "on_key":
            want_masks = (str(self.keyboard_state).strip().lower() == self.mask_publish_key)
        combined_mask_2d = None
        combined_mask_2d_visual = None

        masks = results[0].masks if want_masks else None
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
                    obj_pos_x = self._as_float((results[0].boxes.xyxy[i][0] + results[0].boxes.xyxy[i][2]) / 2.0)
                    obj_pos_y = self._as_float((results[0].boxes.xyxy[i][1] + results[0].boxes.xyxy[i][3]) / 2.0)

                    if gripper_pos_x is not None and obj_pos_x is not None:
                        if self.keyboard_state == "a" and gripper_pos_x < obj_pos_x:  # left
                            class_id = 1
                        elif self.keyboard_state == "d" and gripper_pos_x > obj_pos_x:  # right
                            class_id = 1
                        elif (
                            self.keyboard_state == "Neutral"
                            and class_name == "object"
                            and (abs(gripper_pos_x - obj_pos_x) > 40)
                        ):
                            class_id = 1

                    # Position info for auto grasp and release
                    grasp_executed = rospy.get_param('/grasp_executed', False)
                    release_executed = rospy.get_param('/release_executed', False)

                    if (
                        self.keyboard_state == "Neutral"
                        and class_name == "object"
                        and class_id != 1
                        and gripper_pos_y is not None
                        and obj_pos_y is not None
                        and (abs(gripper_pos_y - obj_pos_y) < 8)
                    ):
                        if not grasp_executed:
                            self.automove_pub.publish("Grasp")

                    elif (
                        self.keyboard_state == "Neutral"
                        and class_name == "bin"
                        and class_id != 1
                        and gripper_pos_y is not None
                        and obj_pos_y is not None
                        and (abs(gripper_pos_y - obj_pos_y) < 2)
                    ):
                        if not release_executed:
                            self.automove_pub.publish("Release")

                resized_mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
                combined_mask_2d[resized_mask > 0] = class_id

                # Attach per-instance mask yaw into JSON detections (if enabled).
                # This is purely image-based and is used by PRIME to implement ALIGN_YAW.
                if self.compute_mask_yaw and i < len(detections) and class_name == "object":
                    try:
                        ys, xs = np.nonzero(resized_mask > 0)
                        n = int(xs.size)
                        if n >= self.mask_yaw_min_pixels:
                            x_mean = float(xs.mean())
                            y_mean = float(ys.mean())
                            x0 = xs.astype(np.float32) - x_mean
                            y0 = ys.astype(np.float32) - y_mean
                            # 2x2 covariance
                            cxx = float(np.mean(x0 * x0))
                            cyy = float(np.mean(y0 * y0))
                            cxy = float(np.mean(x0 * y0))
                            cov = np.array([[cxx, cxy], [cxy, cyy]], dtype=np.float32)
                            eigvals, eigvecs = np.linalg.eigh(cov)  # ascending
                            # principal axis = eigenvector with largest eigenvalue
                            v = eigvecs[:, int(np.argmax(eigvals))]
                            vx, vy = float(v[0]), float(v[1])
                            yaw = float(np.arctan2(vy, vx))  # image coords
                            # elongation ratio (stability heuristic)
                            ev_min = float(max(1e-9, np.min(eigvals)))
                            ev_max = float(max(1e-9, np.max(eigvals)))
                            ratio = float(ev_max / ev_min)
                            if ratio >= float(self.mask_yaw_min_ratio):
                                detections[i]["mask_center_xy"] = [int(round(x_mean)), int(round(y_mean))]
                                detections[i]["mask_yaw_rad"] = yaw
                                detections[i]["mask_yaw_ratio"] = ratio

                                # Optional debug overlay: draw principal axis on the grid image.
                                if self.draw_mask_yaw_overlay:
                                    cx_i = int(round(x_mean))
                                    cy_i = int(round(y_mean))
                                    L = int(max(10, self.mask_yaw_overlay_len_px))
                                    dx = int(round(L * float(np.cos(yaw))))
                                    dy = int(round(L * float(np.sin(yaw))))
                                    p1 = (int(cx_i - dx), int(cy_i - dy))
                                    p2 = (int(cx_i + dx), int(cy_i + dy))
                                    cv2.line(image_with_grid, p1, p2, (0, 255, 255), 2)
                                    cv2.circle(image_with_grid, (cx_i, cy_i), 3, (0, 255, 255), -1)
                    except Exception:
                        # Never break detection publishing due to yaw estimation
                        pass

            combined_mask_2d_visual = np.zeros((height, width, 3), dtype=np.uint8)
            for class_id, color in self.color_map.items():
                combined_mask_2d_visual[combined_mask_2d == class_id] = color

        # Publish debug images + structured detections (always)
        try:
            ros_image_with_dots = self.bridge.cv2_to_imgmsg(image_with_dots, "bgr8")
            self.image_with_dots_pub.publish(ros_image_with_dots)

            ros_image_with_grid = self.bridge.cv2_to_imgmsg(image_with_grid, "bgr8")
            self.image_with_grid_pub.publish(ros_image_with_grid)

            ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.image_pub.publish(ros_image)

            # Publish masks only when explicitly enabled and computed
            if combined_mask_2d is not None and self.mask_2d_pub is not None:
                ros_mask_2d = self.bridge.cv2_to_imgmsg(combined_mask_2d, "mono8")
                self.mask_2d_pub.publish(ros_mask_2d)
            if combined_mask_2d_visual is not None and self.mask_2d_visual_pub is not None:
                ros_mask_2d_visual = self.bridge.cv2_to_imgmsg(combined_mask_2d_visual, "bgr8")
                self.mask_2d_visual_pub.publish(ros_mask_2d_visual)

            payload = {
                "stamp": rospy.Time.now().to_sec(),
                "workspace_bbox_xyxy": workspace_bbox,
                "grid_bbox_xyxy": grid_bbox,
                "grid_crop_top_ratio": self.grid_crop_top_ratio,
                "grid_crop_bottom_ratio": self.grid_crop_bottom_ratio,
                "grid_rows": 3,
                "grid_cols": 3,
                "detections": detections,
            }
            self.detections_pub.publish(String(data=json.dumps(payload)))

        except CvBridgeError as e:
            rospy.logerr(e)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = YoloRosNode()
    node.run()
