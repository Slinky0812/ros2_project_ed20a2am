#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
from nav2_msgs.action import NavigateToPose
import signal
from rclpy.action import ActionClient
from math import sin, cos


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None  # Track active goal
        
        # Initialise a publisher to publish messages to the robot base
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz

        # Initialise any flags that signal a colour has been detected (default to false)
        self.blue_found = False

        # Initialise the value for sensitivity in the colour detection
        self.sensitivity = 10

        # Initialise standard movement flags
        self.moveForwardsFlag = False
        self.moveBackwardsFlag = False
        self.turnLeftFlag = False
        self.turnRightFlag = False
        self.stopFlag = False

        # Initialise a CvBridge()
        self.bridge = CvBridge()

        # Set up a subscriber to the image topic you wish to use
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        self.navigation_mode = True  # Start in navigation mode


    def callback(self, data):
        try:
            # Convert the received image into a opencv image
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            height, width, _ = image.shape  # Get image dimensions
            image_center_x = width // 2  # Middle of the image

            # Set the upper and lower bounds for blue colour
            
            # blue bounds
            hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
            hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])

            # Convert the rgb image into a hsv image
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Filter out everything but the blue colour

            blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

            # Apply the mask to the original image using the cv2.bitwise_and() method
            filtered_img = cv2.bitwise_and(image, image, mask=blue_mask)

            # Find the contours that appear within the certain colour mask using the cv2.findContours() method
            contours, _ = cv2.findContours(blue_mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)

            # Reset flags
            self.blue_found = False
            self.moveForwardsFlag = False
            self.moveBackwardsFlag = False
            self.turnLeftFlag = False
            self.turnRightFlag = False
            self.stopFlag = False

            # Loop over the contours
            if len(contours) > 0:

                # Use the max() method to find the largest contour                
                c = max(contours, key=cv2.contourArea)

                # Moments can calculate the center of the contour
                M = cv2.moments(c)

                if M['m00'] > 0 and cv2.contourArea(c) > 30:  # Avoid division by zero
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # Check if the area of the shape you want is big enough to be considered
                    # if cv2.contourArea(c) > 30:

                    if self.navigation_mode:
                        print("here")
                        # Cancel current navigation goal
                        if self.current_goal_handle and self.current_goal_handle.status == 1:
                            future = self.current_goal_handle.cancel_goal_async()
                            future.add_done_callback(self.cancel_done_callback)
                            self.current_goal_handle = None  # Reset after canceling
                            time.sleep(1)

                    self.navigation_mode = False
                        # # Alter the value of the flag
                        # self.blue_found = True

                    # # Check if blue has been found
                    # if self.blue_found:
                        # Too close to object, need to move backwards
                    if cv2.contourArea(c) > 232000:
                        print("too close")
                        # Set a flag to tell the robot to move backwards when in the main loop
                        self.moveBackwardsFlag = True
                        self.moveForwardsFlag = False
                        self.stopFlag = False
                    # Too far away from object, need to move forwards
                    elif cv2.contourArea(c) < 222000:
                        print("too far")
                        # Set a flag to tell the robot to move forwards when in the main loop
                        self.moveForwardsFlag = True
                        self.moveBackwardsFlag = False
                        self.stopFlag = False
                    # Perfect distance, don't move
                    else:
                        print("perfect??")
                        # Set a flag to tell the robot to stop moving when in the main loop
                        self.stopFlag = True
                        self.moveForwardsFlag = False
                        self.moveBackwardsFlag = False
                        self.turnLeftFlag = False
                        self.turnRightFlag = False
            
                    # Direction check
                    margin = 50  # Tolerance for centering
                    if cx < image_center_x - margin:
                        print("Turning left")
                        self.turnLeftFlag = True
                        self.turnRightFlag = False
                        self.stopFlag = False
                    elif cx > image_center_x + margin:
                        print("Turning right")
                        self.turnRightFlag = True
                        self.turnLeftFlag = False
                        self.stopFlag = False
                    else:
                        self.turnRightFlag = False
                        self.turnLeftFlag = False
                        print("Centered")
                    
                    print(f"area = {cv2.contourArea(c)}")
                # else:
                #     self.navigation_mode = True
                #     self.send_goal(-0.419, -10.1, 2.50)  # Resume navigation

            # Show the resultant images created
            cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
            cv2.imshow('camera_Feed', image)
            cv2.resizeWindow('camera_Feed',320,240)
            cv2.waitKey(3)
            
        except CvBridgeError as e:
            print(e)

        return


    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Successfully canceled goal')
            self.current_goal_handle = None
            self.navigation_mode = False
        else:
            self.get_logger().warning('Failed to cancel goal')


    def send_goal(self, x, y, yaw):
        if self.current_goal_handle is not None:
            self.get_logger().info('A goal is already active')
            return

        if not self.action_client.wait_for_server(timeout_sec=5.0):  # Add timeout
            self.get_logger().error("Action server not available!")
            return
        
        print("in send_goal function")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        # self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, (x, y, yaw))
        )


    def goal_response_callback(self, future, goal_pose):
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.get_logger().info('Goal rejected')
        else:
            self.get_logger().info('Goal accepted')
            self.get_result_future = self.current_goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')



    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # NOTE: if you want, you can use the feedback while the robot is moving.
        #       uncomment to suit your need.

        ## Access the current pose
        #current_pose = feedback_msg.feedback.current_pose
        #position = current_pose.pose.position
        #orientation = current_pose.pose.orientation

        ## Access other feedback fields
        #navigation_time = feedback_msg.feedback.navigation_time
        #distance_remaining = feedback_msg.feedback.distance_remaining

        ## Print or process the feedback data
        #self.get_logger().info(f'Current Pose: [x: {position.x}, y: {position.y}, z: {position.z}]')
        #self.get_logger().info(f'Distance Remaining: {distance_remaining}')


    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2  # Forward with 0.2 m/s

        for _ in range(15):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()


    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2  # Backward with 0.2 m/s
        for _ in range(15):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()


    def turn_left(self):
        desired_velocity = Twist()
        desired_velocity.angular.z = 0.2  # Positive for counter-clockwise rotation
        for _ in range(15):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()



    def turn_right(self):
        desired_velocity = Twist()
        desired_velocity.angular.z = -0.2  # Negative for clockwise rotation
        for _ in range(15):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()



    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        desired_velocity.angular.z = 0.0
        self.publisher.publish(desired_velocity)


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

    goals = [
        # inside small box
        (5.19, 4.03, 0.00),
        (-0.419, -10.1, 2.50) # good point
    ]

    goalIndex = 0

    try:
        while rclpy.ok():

            if robot.current_goal_handle is None and goalIndex < len(goals):
                x, y, yaw = goals[goalIndex]
                robot.send_goal(x, y, yaw)
                goalIndex += 1
            else:
                # Publish moves
                if robot.moveForwardsFlag:
                    robot.walk_forward()

                if robot.moveBackwardsFlag:
                    robot.walk_backward()

                if robot.turnLeftFlag:
                    robot.turn_left()

                if robot.turnRightFlag:
                    robot.turn_right()

                if robot.stopFlag:
                    robot.stop()

            time.sleep(0.1)
            # pass
            continue
    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
