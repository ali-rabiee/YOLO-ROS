#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import threading
import sys
import termios
import tty

class VideoRecorder:
    def __init__(self):
        rospy.init_node('video_recorder', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        
        # Define the default base path
        default_base_path = os.path.expanduser('/home/tnlab/Data/robot/')
        
        # Get the additional path from ROS parameter
        additional_path = rospy.get_param('~output_dir', '')

        # Combine the base path with the additional path
        self.output_dir = os.path.join(default_base_path, additional_path)

        if not os.path.isabs(self.output_dir):
            self.output_dir = os.path.abspath(self.output_dir)

        if not os.path.exists(self.output_dir):
            try:
                os.makedirs(self.output_dir)
            except PermissionError as e:
                rospy.logerr(f"Permission error creating directory {self.output_dir}: {e}")
                raise

        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = None
        
        self.recording = False
        self.latest_frame = None

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.latest_frame = cv_image
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        if self.recording and self.latest_frame is not None:
            if self.out is None:
                height, width = self.latest_frame.shape[:2]

                # Get the next available video number
                video_number = self.get_next_video_number()

                # Format the output filename with leading zeros (e.g., 004.mp4)
                self.output_filename = os.path.join(self.output_dir, f'{video_number:03d}.mp4')
                
                self.out = cv2.VideoWriter(self.output_filename, self.fourcc, 30, (width, height))
            
            self.out.write(self.latest_frame)

    def get_next_video_number(self):
        # List all files in the output directory
        existing_files = os.listdir(self.output_dir)

        # Filter out only .mp4 files and extract the numbers
        video_numbers = [int(f.split('.')[0]) for f in existing_files if f.endswith('.mp4') and f.split('.')[0].isdigit()]

        # If there are no videos, start with 1
        if not video_numbers:
            return 1

        # Return the next number
        return max(video_numbers) + 1

    def start_recording(self):
        if not self.recording:
            self.recording = True
            rospy.loginfo("Started recording")

    def stop_recording(self):
        if self.recording:
            self.recording = False
            if self.out is not None:
                self.out.release()
                self.out = None
            rospy.loginfo("Stopped recording")

    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def keyboard_control(self):
        rospy.loginfo("Press 'a' to start recording, 's' to stop recording, 'q' to quit")
        while not rospy.is_shutdown():
            c = self.getch()
            if c == 'a':
                self.start_recording()
            elif c == 's':
                self.stop_recording()
            elif c == 'q':
                rospy.signal_shutdown("User requested shutdown")
                break

    def run(self):
        keyboard_thread = threading.Thread(target=self.keyboard_control)
        keyboard_thread.start()
        rospy.spin()
        keyboard_thread.join()

if __name__ == '__main__':
    recorder = VideoRecorder()
    recorder.run()
