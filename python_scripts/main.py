import rospy
import threading
from exploration import *
from uav import *

def user_input(drone_sub):
    while not rospy.is_shutdown():
        cmd = input("Give a command: ")
        if cmd == "q":
            print("Quitting")
        elif cmd == "w":
            drone_sub.pos.x += 1
            drone_sub.pos.z = drone_sub.altitude
            send_pose(drone_sub.pos, drone_sub.orient)
        elif cmd == "start":
            points = generate_points_within_polygon(drone_sub.angles, step=5)
            plot_polygon_and_points(drone_sub.angles, points)
            drone_sub.follow_waypoints(points)
        elif cmd == "go":
            drone_sub.move_to(drone_sub.angles[-1])
        elif cmd == "show":
            points = generate_points_within_polygon(drone_sub.angles, step=5)
            plot_polygon_and_points(drone_sub.angles, points)
        elif cmd == "delete markers":
            drone_sub.delete_all_markers()
        else:
            try:
                pos = list(map(float, cmd.split()))
                drone_sub.go(pos)
            except:
                print("Invalid command")

if __name__ == '__main__':
    try:
        drone_sub = DroneSubscriber()
        threading.Thread(target=user_input, args=(drone_sub,)).start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Destroy any OpenCV windows when the script is stopped
        cv2.destroyAllWindows()