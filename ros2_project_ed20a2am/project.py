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

        # Initiate action client
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Track active goal
        self.current_goal_handle = None
        
        # Keep track of completed goals
        self.goals_completed = 0
        
        # Start in navigation mode
        self.navigation_mode = True

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


    def callback(self, data):
        try:
            # Convert the received image into a opencv image
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            height, width, _ = image.shape  # Get image dimensions
            image_center_x = width // 2  # Middle of the image

            # Set the upper and lower bounds for rgb colours
            
            # blue bounds
            hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
            hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])

            # green bounds
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

            # red lower bounds
            hsv_red_lower_1 = np.array([0 - self.sensitivity, 100, 100])
            hsv_red_upper_1 = np.array([0 + self.sensitivity, 255, 255])

            # red upper bounds
            hsv_red_lower_2 = np.array([180 - self.sensitivity, 100, 100])
            hsv_red_upper_2 = np.array([180 + self.sensitivity, 255, 255])

            # Convert the rgb image into a hsv image
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Create masks for rgb

            # blue mask
            blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

            # green mask
            green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

            # red mask
            red_mask_1 = cv2.inRange(hsv_image, hsv_red_lower_1, hsv_red_upper_1)

            red_mask_2 = cv2.inRange(hsv_image, hsv_red_lower_2, hsv_red_upper_2)

            red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

            # rgb mask
            gb_mask = cv2.bitwise_or(green_mask, blue_mask)

            rgb_mask = cv2.bitwise_or(red_mask, gb_mask)

            # Apply the mask to the original image using the cv2.bitwise_and() method
            filtered_img = cv2.bitwise_and(image, image, mask=rgb_mask)

            # Find the contours that appear within the blue colour mask using the cv2.findContours() method
            contours, _ = cv2.findContours(blue_mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)

            # Reset flags
            self.blue_found = False
            self.moveForwardsFlag = False
            self.moveBackwardsFlag = False
            self.turnLeftFlag = False
            self.turnRightFlag = False
            self.stopFlag = False

            # Loop over the contours for the blue mask
            if len(contours) > 0:

                # Use the max() method to find the largest contour                
                c = max(contours, key=cv2.contourArea)

                # Moments can calculate the center of the contour
                M = cv2.moments(c)

                # Check if the area of the shape you want is big enough to be considered
                if M['m00'] > 0 and cv2.contourArea(c) > 10000:  # Avoid division by zero
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    x, y, w, h = cv2.boundingRect(c)  # Get bounding rectangle
                    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Draw blue rectangle with thickness 2

                    # Cancel current navigation goal
                    if self.navigation_mode:
                        if self.current_goal_handle and self.current_goal_handle.status == 2:
                            self.get_logger().info('Found blue object')
                            future = self.current_goal_handle.cancel_goal_async()
                            future.add_done_callback(self.cancel_done_callback)
                            time.sleep(1)

                    # Too close to object, need to move backwards
                    if cv2.contourArea(c) > 232000:
                        # Set a flag to tell the robot to move backwards when in the main loop
                        self.moveBackwardsFlag = True
                        self.moveForwardsFlag = False
                        self.stopFlag = False
                    # Too far away from object, need to move forwards
                    elif cv2.contourArea(c) < 222000:
                        # Set a flag to tell the robot to move forwards when in the main loop
                        self.moveForwardsFlag = True
                        self.moveBackwardsFlag = False
                        self.stopFlag = False
                    # Perfect distance, don't move
                    else:
                        # Set a flag to tell the robot to stop moving when in the main loop
                        self.stopFlag = True
                        self.moveForwardsFlag = False
                        self.moveBackwardsFlag = False
                        self.turnLeftFlag = False
                        self.turnRightFlag = False
            
                    # Direction check
                    margin = 50  # Tolerance for centering
                    # Too much to the right, need to turn left
                    if cx < image_center_x - margin - 10:
                        # Set a flag to tell the robot to turn left when in the main loop
                        self.turnLeftFlag = True
                        self.turnRightFlag = False
                        self.stopFlag = False
                    # Too much to the left, need to turn right
                    elif cx > image_center_x + margin + 10:
                        # Set a flag to tell the robot to turn rightwhen in the main loop
                        self.turnRightFlag = True
                        self.turnLeftFlag = False
                        self.stopFlag = False
                    else:
                        self.turnRightFlag = False
                        self.turnLeftFlag = False

            # Find the contours that appear within the green colour mask using the cv2.findContours() method
            contours, _ = cv2.findContours(green_mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)

            # Loop over the contours for the green mask
            if len(contours) > 0:
                # Use the max() method to find the largest contour                
                c = max(contours, key=cv2.contourArea)

                # Moments can calculate the center of the contour
                M = cv2.moments(c)

                # Check if the area of the shape you want is big enough to be considered
                if M['m00'] > 0 and cv2.contourArea(c) > 100:
                    cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

                    x, y, w, h = cv2.boundingRect(c)  # Get bounding rectangle
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw green rectangle with thickness 2

            # Find the contours that appear within the red colour mask using the cv2.findContours() method
            contours, _ = cv2.findContours(red_mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)

            # Loop over the contours for the red mask
            if len(contours) > 0:
                # Use the max() method to find the largest contour                
                c = max(contours, key=cv2.contourArea)

                # Moments can calculate the center of the contour
                M = cv2.moments(c)

                # Check if the area of the shape you want is big enough to be considered
                if M['m00'] > 0 and cv2.contourArea(c) > 100:
                    cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

                    x, y, w, h = cv2.boundingRect(c)  # Get bounding rectangle
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)  # Draw red rectangle with thickness 2

            # Show the resultant images created
            cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
            cv2.imshow('camera_Feed', image)
            cv2.resizeWindow('camera_Feed',320,240)
            cv2.waitKey(3)
            
        except CvBridgeError as e:
            print(e)

        return


    def cancel_done_callback(self, future):        
        # Cancel current goal
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Successfully canceled goal')
            self.current_goal_handle = None
            self.navigation_mode = False
        else:
            self.get_logger().warning('Failed to cancel goal')


    def send_goal(self, x, y, yaw):
        # Check if goal is currently being tracked
        if self.current_goal_handle is not None:
            self.get_logger().info('A goal is already active')
            return

        # Check if action client (RViz) is working
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return
        
        self.get_logger().info(f"Sending goal: x={x}, y={y}, yaw={yaw}")
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
        self.send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, (x, y, yaw)))


    def goal_response_callback(self, future, goal_pose):
        # Check if goal is accepted or not
        current_goal_handle = future.result()
        if not current_goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.current_goal_handle = None
        else:
            self.get_logger().info('Goal accepted')
            self.current_goal_handle = current_goal_handle
            self.get_result_future = self.current_goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        # Goal is complete
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.current_goal_handle = None  # Allow new goal only after completion
        self.goals_completed += 1
        self.get_logger().info(f'Goals completed: {self.goals_completed}')


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback


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
        self.get_logger().info('Perfect distance from blue box - Stopped moving')


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

    # list of goals
    goals = [
        (5.19, 4.03, 0.00),
        (-4.61, 0.153, 3.14),
        (2.5, -7.96, 4.50),
        (-0.419, -10.1, 2.50)
    ]

    goalIndex = 0

    try:
        while rclpy.ok():
            # if robot is tracking a goal
            if robot.navigation_mode:
                # if we have finished tracking a goal, track the next goal
                if robot.current_goal_handle is None and goalIndex < len(goals) and robot.goals_completed == goalIndex:
                    x, y, yaw = goals[goalIndex]
                    robot.get_logger().info(f"Sending goal {goalIndex+1}/{len(goals)}: ({x}, {y}, {yaw})")
                    robot.send_goal(x, y, yaw)
                    goalIndex += 1
            # if robot has seen the blue box
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
            continue
    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
