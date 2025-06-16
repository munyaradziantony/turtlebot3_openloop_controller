#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import csv

class OpenLoopControllerWithAcceleration(Node):
    def __init__(self):
        super().__init__('tb_openLoop_with_acc_and_decc')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()

        # Motion parameters
        self.max_speed = 2.0  # m/s
        self.total_distance = 10.0  # m
        self.acceleration = 1.0  # m/s²

        # Compute distances and times for each phase
        self.accel_distance = (self.max_speed ** 2) / (2 * self.acceleration)
        self.decel_distance = self.accel_distance
        self.constant_distance = self.total_distance - self.accel_distance - self.decel_distance

        self.accel_time = self.max_speed / self.acceleration
        self.constant_time = self.constant_distance / self.max_speed
        self.decel_time = self.accel_time
        self.total_time = self.accel_time + self.constant_time + self.decel_time

        # Tracking variables
        self.elapsed_time = 0.0
        self.current_distance = 0.0
        self.time_data = []
        self.distance_data = []
        self.speed_data = []
        self.speed = Twist()

        self.get_logger().info(
            f"Phases:\n"
            f"  Acceleration: {self.accel_time:.2f}s over {self.accel_distance:.2f}m\n"
            f"  Constant:     {self.constant_time:.2f}s over {self.constant_distance:.2f}m\n"
            f"  Deceleration: {self.decel_time:.2f}s over {self.decel_distance:.2f}m"
        )

    def timer_callback(self):
        current_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.elapsed_time += self.timer_period

        if current_time < self.accel_time:
            # Phase 1: Accelerating
            self.speed.linear.x = self.acceleration * current_time
            self.get_logger().info(f'Accelerating: {self.speed.linear.x:.2f} m/s')

        elif current_time < self.accel_time + self.constant_time:
            # Phase 2: Constant speed
            self.speed.linear.x = self.max_speed
            self.get_logger().info(f'Cruising: {self.speed.linear.x:.2f} m/s')

        elif current_time < self.total_time:
            # Phase 3: Decelerating
            t_decel = current_time - self.accel_time - self.constant_time
            self.speed.linear.x = max(0.0, self.max_speed - self.acceleration * t_decel)
            self.get_logger().info(f'Decelerating: {self.speed.linear.x:.2f} m/s')

        else:
            # Stop and finalize
            self.speed.linear.x = 0.0
            self.publisher_.publish(self.speed)
            self.get_logger().info('Reached target. Stopping robot.')
            self.plot_graphs()
            self.save_to_csv()
            self.destroy_node()
            return

        # Distance integration
        self.current_distance += self.speed.linear.x * self.timer_period

        # Log data
        self.time_data.append(self.elapsed_time)
        self.distance_data.append(self.current_distance)
        self.speed_data.append(self.speed.linear.x)

        # Publish command
        self.publisher_.publish(self.speed)

    def plot_graphs(self):
        plt.figure(figsize=(12, 5))

        # Distance vs Time
        plt.subplot(1, 2, 1)
        plt.plot(self.time_data, self.distance_data, 'b-o')
        plt.title('Distance vs Time for acc and decc')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.grid()

        # Speed vs Time
        plt.subplot(1, 2, 2)
        plt.plot(self.time_data, self.speed_data, 'r-o')
        plt.title('Speed vs Time for acc and decc')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.grid()

        plt.tight_layout()
        plt.savefig('motion_profile for acc and decc.png')
        plt.show()

    def save_to_csv(self):
        with open('distance_speed_time_data_acc_decc.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'Distance (m)', 'Speed (m/s)'])
            for t, d, v in zip(self.time_data, self.distance_data, self.speed_data):
                writer.writerow([t, d, v])
        self.get_logger().info('Data saved to distance_speed_time_data_acc_decc.csv.')

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopControllerWithAcceleration()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



