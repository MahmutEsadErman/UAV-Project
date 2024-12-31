#!/usr/bin/env python
import time
import math
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Point, PoseWithCovarianceStamped, Quaternion
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations

import cv2
from cv_bridge import CvBridge, CvBridgeError

# my libraries
from exploration import *

def sign(n): 
    if n<0: return -1 
    elif n>0: return 1 
    else: return 0 

def send_pose(pos, orient):
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
        self.pos = PoseStamped()
        self.orient = Quaternion()
        self.altitude = 5.0

        self.marker_no = 0
        self.marker_array = MarkerArray()
        self.angles = []
        self.waypoints = []

        # Initialize the ROS node
        rospy.init_node('drone_subscriber', anonymous=True)
        
        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/firefly/vi_sensor/right/image_raw', Image, self.image_callback)

        # Subscribe to the drone position topic
        self.drone_pos_sub = rospy.Subscriber('/firefly/ground_truth/pose_with_covariance', PoseWithCovarianceStamped, self.drone_pos_callback)
        
        # Subscribe to the clicked point topic
        self.click_sub = rospy.Subscriber('/clicked_point', PointStamped, self.click_callback)

        # Create a publisher for the markers
        self.markers_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=100)

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = CvBridge().imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return
        
        # Display the image in a window
        cv2.imshow('Camera Image', cv_image)
        cv2.waitKey(1)  # Press any key to close the window

    def drone_pos_callback(self, msg):
        self.pos = msg.pose.pose.position
        self.orient = msg.pose.pose.orientation

    def click_callback(self, msg):
        print("clicked coordinates: x = %f y = %f" %(msg.point.x, msg.point.y))
        if len(self.angles) > 0:
            self.put_marker(msg.point, Point(self.angles[-1][0], self.angles[-1][1], 0))
        else:
            self.put_marker(msg.point)
        self.angles.append([msg.point.x,msg.point.y])

    def follow_waypoints(self, waypoints, step_size=1):
        print("Following Waypoints Mission Started")
        for wp in waypoints:
            self.move_to(wp, 0.5, step_size)
        print("Waypoints Mission Completed")
    
    def move_to(self, pos, radius=0.5, step_size=1):
        # Calculate orientation
        dx = pos[0] - self.pos.x
        dy = pos[1] - self.pos.y
        yaw = math.atan2(dy, dx)

        # Convert yaw to quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        orient = Quaternion()
        orient.x = quaternion[0]
        orient.y = quaternion[1]
        orient.z = quaternion[2]
        orient.w = quaternion[3]

        while(abs(pos[0]-self.pos.x) > radius or abs(pos[1]-self.pos.y) > radius):
            self.pos.x += sign(pos[0]-self.pos.x)*step_size
            self.pos.y += sign(pos[1]-self.pos.y)*step_size
            self.pos.z = self.altitude
            
            send_pose(self.pos, orient)
            time.sleep(0.1)
        print("Reached the target position: ", self.pos.x, self.pos.y)
    
    def put_marker(self, pos, last_pos=None):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time()
        marker.ns = "marker"+str(self.marker_no)
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = pos
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 25.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.lifetime = rospy.Duration()

        if last_pos != None:
            line = Marker()
            line.header.frame_id = "base_link"
            line.header.stamp = rospy.Time()
            line.ns = "line"+str(self.marker_no)
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.x = 0.0
            line.pose.orientation.y = 0.0
            line.pose.orientation.z = 0.0
            line.pose.orientation.w = 1.0
            line.scale.x = 1.0
            line.scale.y = 1.0
            line.scale.z = 1.0
            line.color.a = 1.0
            line.color.r = 0.0
            line.color.g = 1.0
            line.color.b = 0.0
            line.lifetime = rospy.Duration()
            line.points.append(last_pos)
            line.points.append(pos)
            self.marker_no += 2
            self.marker_array.markers.append(line)
        else:
            self.marker_no += 1

        
        self.marker_array.markers.append(marker)
        self.markers_pub.publish(self.marker_array)
        
    def delete_all_markers(self):
        self.angles = []
        for marker in self.marker_array.markers:
            marker.action = Marker.DELETE
        self.markers_pub.publish(self.marker_array)
        print("All markers deleted")


if __name__ == '__main__':
    try:
        drone_sub = DroneSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Destroy any OpenCV windows when the script is stopped
        cv2.destroyAllWindows()