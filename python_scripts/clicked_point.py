#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray

import cv2
from cv_bridge import CvBridge, CvBridgeError

import threading

def sign(n): 
    if n<0: return -1 
    elif n>0: return 1 
    else: 0 

def send_pose(pos, orient):
	pose_pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=1)

	rate = rospy.Rate(100)  # 10 Hz
	pose_msg = PoseStamped()
	
	# Set frame and time
	pose_msg.header.frame_id = "world"
	
	# Set position (x, y, z)
	pose_msg.pose.position = pos

	# Set orientation (Quaternion)
	#pose_msg.pose.orientation = orient

	pose_msg.header.stamp = rospy.Time.now()  # Update timestamp
	pose_pub.publish(pose_msg)
	rate.sleep()



class DroneSubscriber:
    def __init__(self):
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.altitude = 5.0

        self.marker_no = 0
        self.markers_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=100)
        self.marker_array = MarkerArray()
        self.waypoints = []

        # Initialize the ROS node
        rospy.init_node('drone_subscriber', anonymous=True)
        
        # Create a CvBridge object to convert ROS images to OpenCV images
        self.bridge = CvBridge()
        
        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/firefly/vi_sensor/right/image_raw', Image, self.image_callback)

        self.states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.states_callback)
        
        self.click = rospy.Subscriber('/clicked_point', PointStamped, self.click_callback)

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

    def click_callback(self, msg):
        print("coordinates:x=%f y=%f" %(msg.point.x, msg.point.y))
        if len(self.waypoints) > 0:
            self.put_marker(msg.point, Point(self.waypoints[-1][0], self.waypoints[-1][1], 0))
        else:
            self.put_marker(msg.point)
        self.waypoints.append([msg.point.x,msg.point.y])

    def follow_waypoints(self):
        print("Following Waypoints Mission Started")
        for i, wp in enumerate(self.waypoints):
            print(i+1, ". waypoint: ", wp)
            self.go(wp)
            print("reached", i+1, ". waypoint: ")
        
        print("Waypoints Mission Completed")
    
    def go(self, pos):
        eps = 1.5
        radius = 0.1
        while(abs(pos[0]-self.drone_pos.x) > radius and abs(pos[1]-self.drone_pos.y) > radius):
            self.drone_pos.x += sign(pos[0]-self.drone_pos.x)*eps
            self.drone_pos.y += sign(pos[1]-self.drone_pos.y)*eps
            self.drone_pos.z = self.altitude
            send_pose(self.drone_pos, self.drone_orient)
            time.sleep(0.1)


    def user_input(self):
        while not rospy.is_shutdown():
            cmd = input("Give a command: ")
            if cmd == "q":
                print("Quitting")
            elif cmd == "w":
                self.drone_pos.x += 1
                self.drone_pos.z = self.altitude
                send_pose(self.drone_pos, self.drone_orient)
            elif cmd == "start":
                self.follow_waypoints()
            elif cmd == "go":
                self.go_waypoint()
            else:
                cmd = list(map(float, cmd.split()))
                self.go(cmd)
    
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
        marker.scale.z = 50.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration()

        if last_pos != None:
            line = Marker()
            line.header.frame_id = "base_link"
            line.header.stamp = rospy.Time()
            line.ns = "line"+str(self.marker_no)
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            #line.pose.position = pos
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
        
        print("Marker published")


if __name__ == '__main__':
    try:
        drone_sub = DroneSubscriber()
        
        threading.Thread(target=drone_sub.user_input).start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Destroy any OpenCV windows when the script is stopped
        cv2.destroyAllWindows()
