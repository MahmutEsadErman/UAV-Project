#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_from_euler, quaternion_multiply

from cv_bridge import CvBridge, CvBridgeError

import cv2
from pynput.keyboard import Key, Listener
import math


def send_pose(pos, orient):
    # rospy.init_node('drone_pose_publisher', anonymous=True)
    pose_pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=1)

    rate = rospy.Rate(100)  # 10 Hz
    pose_msg = PoseStamped()
    
    # Set frame and time
    pose_msg.header.frame_id = "world"
    
    # Set position (x, y, z)
    pose_msg.pose.position = pos

    # Set orientation (Quaternion)
    pose_msg.pose.orientation = orient

    pose_msg.header.stamp = rospy.Time.now()  # Update timestamp
    pose_pub.publish(pose_msg)
    rate.sleep()

class DroneSubscriber:
    def __init__(self):
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0

        # Initialize the ROS node
        rospy.init_node('camera_subscriber', anonymous=True)
        
        # Create a CvBridge object to convert ROS images to OpenCV images
        self.bridge = CvBridge()
        
        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/firefly/camera_nadir/image_raw', Image, self.image_callback)

        self.states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.states_callback)

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return
        
        # Display the image in a window
        cv2.imshow('Camera Image', cv_image)
        cv2.waitKey(1)  # Press any key to close the window

    def states_callback(self, data):
        self.drone_pos = data.pose[1].position
        self.drone_orient = data.pose[1].orientation


    def on_press(self, key):
        try:
            # Print only letter keys
            if key.char == 'q':
                cv2.destroyAllWindows()
                rospy.signal_shutdown('Quit')
            if key.char == 'w':
                self.drone_pos.z += 0.7
                send_pose(self.drone_pos, self.drone_orient)
            if key.char == 's':
                self.drone_pos.z -= 0.7
                send_pose(self.drone_pos, self.drone_orient)
            if key.char == 'a':
                self.drone_pos.x -= 0.7
                send_pose(self.drone_pos, self.drone_orient)
            if key.char == 'd':
                self.drone_pos.x += 0.7
                send_pose(self.drone_pos, self.drone_orient)

        except AttributeError:
            # Handle special keys (e.g., ctrl, alt, etc.)
            if key == Key.up:
                self.drone_pos.y += 0.7
                send_pose(self.drone_pos, self.drone_orient)
            if key == Key.down:
                self.drone_pos.y -= 0.7
                send_pose(self.drone_pos, self.drone_orient)
            if key == Key.left:
                angle_radians = -math.pi/4
                rotation_quaternion = quaternion_from_euler(0, 0, angle_radians)
                orient_list = [self.drone_orient.x, self.drone_orient.y, self.drone_orient.z, self.drone_orient.w]
                orient_list = quaternion_multiply(orient_list, rotation_quaternion)
                self.drone_orient.x = orient_list[0]
                self.drone_orient.y = orient_list[1]
                self.drone_orient.z = orient_list[2]
                self.drone_orient.w = orient_list[3]
                send_pose(self.drone_pos, self.drone_orient)
            if key == Key.right:
                angle_radians = math.pi/4
                rotation_quaternion = quaternion_from_euler(0, 0, angle_radians)
                orient_list = [self.drone_orient.x, self.drone_orient.y, self.drone_orient.z, self.drone_orient.w]
                orient_list = quaternion_multiply(orient_list, rotation_quaternion)
                self.drone_orient.x = orient_list[0]
                self.drone_orient.y = orient_list[1]
                self.drone_orient.z = orient_list[2]
                self.drone_orient.w = orient_list[3]
                send_pose(self.drone_pos, self.drone_orient)
        
                

if __name__ == '__main__':
    try:
        drone_sub = DroneSubscriber()
        Listener(on_press = drone_sub.on_press).start()
        # threading.Thread(target=pose_thread).start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Destroy any OpenCV windows when the script is stopped
        cv2.destroyAllWindows()
