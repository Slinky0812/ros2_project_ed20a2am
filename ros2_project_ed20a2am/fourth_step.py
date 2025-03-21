# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        # Initialise a publisher to publish messages to the robot base
        # We covered which topic receives messages that move the robot in the 3rd Lab Session
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz

        # Initialise any flags that signal a colour has been detected (default to false)
        self.blue_found = False
        self.green_found = False

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10

        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
        self.moveForwardsFlag = False
        self.moveBackwardsFlag = False
        self.stopFlag = False

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        # We covered which topic to subscribe to should you wish to receive image data

    def callback(self, data):
        try:
            # Convert the received image into a opencv image
            # But remember that you should always wrap a call to this conversion method in an exception handler
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Set the upper and lower bounds for the two colours you wish to identify
            # hue value = 0 to 179
            
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
            
            hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
            hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])

            # Convert the rgb image into a hsv image
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Filter out everything but a particular colour using the cv2.inRange() method
            green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

            blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

            gb_mask = cv2.bitwise_or(green_mask, blue_mask)

            # Apply the mask to the original image using the cv2.bitwise_and() method
            # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
            filtered_img = cv2.bitwise_and(image, image, mask=gb_mask)

            # Find the contours that appear within the certain colour mask using the cv2.findContours() method
            # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
            contours, _ = cv2.findContours(green_mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)

            self.green_found = False
            self.moveForwardsFlag = False
            self.moveBackwardsFlag = False
            self.stopFlag = False

            # Loop over the contours
            if len(contours)>0:

                # There are a few different methods for identifying which contour is the biggest
                # Loop through the list and keep track of which contour is biggest or
                # Use the max() method to find the largest contour
                
                c = max(contours, key=cv2.contourArea)


                # Check if the area of the shape you want is big enough to be considered
                # If it is then change the flag for that colour to be True(1)
                if cv2.contourArea(c) > 5: #<What do you think is a suitable area?>
                    # Alter the value of the flag
                    self.green_found = True

            #Check if a flag has been set = colour object detected - follow the colour object
            if self.green_found:
                if cv2.contourArea(c) > 5:
                    # Too close to object, need to move backwards
                    # Set a flag to tell the robot to move backwards when in the main loop
                    self.moveBackwardsFlag = True
                    # self.walk_backward()
                    
                elif cv2.contourArea(c) < 5:
                    # Too far away from object, need to move forwards
                    # Set a flag to tell the robot to move forwards when in the main loop
                    self.moveForwardsFlag = True
                    # self.walk_forward()
                else:
                    self.stopFlag = True
                    # self.stop()
                    

                # Be sure to do this for the other colour as well
                # Setting the flag to detect blue, and stop the turtlebot from moving if blue is detected

            contours, _ = cv2.findContours(blue_mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)

            self.blue_found = False
            self.stopFlag = False

            # Loop over the contours
            if len(contours)>0:

                # There are a few different methods for identifying which contour is the biggest
                # Loop through the list and keep track of which contour is biggest or
                # Use the max() method to find the largest contour
                
                c = max(contours, key=cv2.contourArea)


                # Check if the area of the shape you want is big enough to be considered
                # If it is then change the flag for that colour to be True(1)
                if cv2.contourArea(c) > 5: #<What do you think is a suitable area?>
                    # Alter the value of the flag
                    self.blue_found = True

            #Check if a flag has been set = colour object detected - follow the colour object
            if self.blue_found:
                # if cv2.contourArea(c) > 5:
                #     # Too close to object, need to move backwards
                #     # Set a flag to tell the robot to move backwards when in the main loop
                #     self.moveBackwardsFlag = True
                    
                # elif cv2.contourArea(c) < 5:
                #     # Too far away from object, need to move forwards
                #     # Set a flag to tell the robot to move forwards when in the main loop
                #     self.moveForwardsFlag = True
                self.stopFlag = True
                # self.stop()

            # Show the resultant images you have created. You can show all of them or just the end result if you wish to.
            cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
            cv2.imshow('camera_Feed', image)
            cv2.resizeWindow('camera_Feed',320,240)
            cv2.waitKey(3)
            
        except CvBridgeError as e:
            print(e)

        return


    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2  # Forward with 0.2 m/s

        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2  # Backward with 0.2 m/s
        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        self.publisher.publish(desired_velocity)


# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    robot = Robot()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            # Publish moves
            if robot.moveForwardsFlag:
                robot.walk_forward()

            if robot.moveBackwardsFlag:
                robot.walk_backward()

            if robot.stopFlag:
                robot.stop()
            # pass
            continue
    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
