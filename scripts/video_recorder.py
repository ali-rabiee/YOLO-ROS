#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import threading
from pynput import keyboard  # Import for keyboard listener

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
        
        self.latest_frame = None
        self.file_lock = threading.Lock()  # To ensure thread safety for file writing
        
        self.monitored_keys = ['w', 'a', 'd', 'g', 'r']
        self.key_state = {key: False for key in self.monitored_keys}
        self.frequency = 2  # Set the frequency in Hz (e.g., 2 times per second)
        
        self.recording = True  # Start recording by default

        self.timer_thread = threading.Thread(target=self.log_key_state)
        self.timer_thread.daemon = True
        self.timer_thread.start()

        # Automatically start recording
        self.start_recording()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.latest_frame = cv_image
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        if self.latest_frame is not None and self.recording:
            if self.out is None:
                height, width = self.latest_frame.shape[:2]

                # Get the next available video number
                video_number = self.get_next_video_number()

                # Format the output filename with leading zeros (e.g., 004.mp4)
                self.output_filename = os.path.join(self.output_dir, f'{video_number:03d}.mp4')

                # Create the video writer
                self.out = cv2.VideoWriter(self.output_filename, self.fourcc, 30, (width, height))

                # Create a keyboard log file with the same name as the video
                self.keyboard_log_file = os.path.join(self.output_dir, f'{video_number:03d}.txt')
            
            # Write the frame to the video file
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
        # Start recording immediately when the node is launched
        rospy.loginfo("Recording started automatically")

    def stop_recording(self):
        # Stop recording and release the video writer
        if self.recording and self.out is not None:
            self.out.release()
            self.out = None
            self.recording = False
            rospy.loginfo("Recording stopped.")

    def log_key_state(self):
        rate = rospy.Rate(self.frequency)  # Set the desired frequency
        while not rospy.is_shutdown() and self.recording:
            # Determine the active key or 'x' if no key is pressed
            active_keys = [k for k, v in self.key_state.items() if v]
            if active_keys:
                state = ','.join(active_keys)
            else:
                state = 'x'  # Log 'x' when no monitored keys are pressed

            # Write the key state to the log file
            if hasattr(self, 'keyboard_log_file'):
                with self.file_lock:
                    with open(self.keyboard_log_file, 'a') as f:
                        f.write(f"{state}\n")

            rate.sleep()

    def on_press(self, key):
        try:
            char = key.char
            if char in self.monitored_keys:
                self.key_state[char] = True
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            char = key.char
            if char in self.monitored_keys:
                self.key_state[char] = False
        except AttributeError:
            pass

        # Stop recording and exit on pressing 'Esc'
        if key == keyboard.Key.esc:
            self.stop_recording()
            return False  # Stop the listener

    def run(self):
        # Start the keyboard listener in a separate thread
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        rospy.loginfo("Video recorder is running. Press 'Esc' to stop recording and exit.")
        rospy.spin()

        listener.join()

if __name__ == '__main__':
    recorder = VideoRecorder()
    try:
        recorder.run()
    except rospy.ROSInterruptException:
        pass