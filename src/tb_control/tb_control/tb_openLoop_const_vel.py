#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import csv

class OpenLoopControllerConstantSpeed(Node):
    def __init__(self):
        super().__init__('tb_openLoop_const_vel')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Proper Twist message for publishing
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 2.0  # constant linear speed (m/s)

        self.distance = 10.0  # goal distance (m)
        self.speed = self.twist_msg.linear.x  # use float for calculation
        self.time_to_run = self.distance / self.speed  # time required to reach goal
        self.elapsed_time = 0.0

        # Data for plotting
        self.time_data = []
        self.distance_data = []
        self.speed_data = []
        self.current_distance = 0.0

    def timer_callback(self):
        if self.elapsed_time < self.time_to_run:
            self.publisher_.publish(self.twist_msg)  
            self.get_logger().info(f'Moving at constant speed: {self.twist_msg.linear.x:.2f} m/s')

            # Update time and distance
            self.elapsed_time += self.timer_period
            self.current_distance += self.twist_msg.linear.x * self.timer_period

            # Log data
            self.time_data.append(self.elapsed_time)
            self.distance_data.append(self.current_distance)
            self.speed_data.append(self.twist_msg.linear.x)

        else:
            # Stop robot
            self.twist_msg.linear.x = 0.0
            self.publisher_.publish(self.twist_msg)
            self.get_logger().info('Goal distance reached. Stopping the robot.')
            self.timer.cancel()

            # Plot results
            self.plot_graphs()
            plt.savefig('Motion with constant speed.png')
            self.save_to_csv()

    def plot_graphs(self):
        plt.figure(figsize=(12, 5))

        # Distance vs Time
        plt.subplot(1, 2, 1)
        plt.plot( self.time_data, self.distance_data, marker='o', color='b')
        plt.title('Distance vs Time Graph constant velocity')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.grid()

        # Speed vs Time
        plt.subplot(1, 2, 2)
        plt.plot(self.time_data, self.speed_data, marker='o', color='r')
        plt.title('Speed vs Time Graph const velocity')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.grid()

        plt.tight_layout()
        plt.show()

    def save_to_csv(self):
        with open('distance_speed_time_data_const_vel.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'Distance (m)', 'Speed (m/s)'])
            for t, d, v in zip(self.time_data, self.distance_data, self.speed_data):
                writer.writerow([t, d, v])
        self.get_logger().info('Data saved to distance_speed_time_data_const_vel.csv.')

def main(args=None):
    rclpy.init(args=args)
    const_speed_controller = OpenLoopControllerConstantSpeed()
    rclpy.spin(const_speed_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()





