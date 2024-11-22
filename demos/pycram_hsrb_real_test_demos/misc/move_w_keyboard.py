import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import threading

def publish_twist(pub, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = linear_y
    twist.linear.z = linear_z
    twist.angular.x = angular_x
    twist.angular.y = angular_y
    twist.angular.z = angular_z

    pub.publish(twist)

def on_press(key, pub):
    try:
        if key.char == 'w':
            publish_twist(pub, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0)  # Move forward
        elif key.char == 's':
            publish_twist(pub, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0)  # Move backward
        elif key.char == 'a':
            publish_twist(pub, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1)  # Turn left
        elif key.char == 'd':
            publish_twist(pub, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1)  # Turn right
        elif key.char == 'x':
            publish_twist(pub, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # Stop
    except AttributeError:
        pass

def listen_keyboard(pub):
    listener = keyboard.Listener(on_press=lambda key: on_press(key, pub))
    listener.start()
    listener.join()

def main():
    rospy.init_node('teleop_twist_keyboard', anonymous=True)
    pub = rospy.Publisher('/hsrb/command_velocity_teleop', Twist, queue_size=10)

    print("Press 'w' to move forward, 's' to move backward, 'a' to turn left, 'd' to turn right")
    print("Press 'x' to stop")

    keyboard_thread = threading.Thread(target=listen_keyboard, args=(pub,))
    keyboard_thread.start()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
#