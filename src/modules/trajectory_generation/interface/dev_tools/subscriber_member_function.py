import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from common_ros2.msg import Trajectory
import matplotlib.pyplot as plt


# remaining
# add rostopic
# debug

class TrajectoryPlotter(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Trajectory,
            '/trajectory',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s" Trajectory points' % len(msg.trajectory_points))
        x = []
        y = []
        v = []
        t = []
        for state in msg.trajectory_points:
            x.append(state.x)
            y.append(state.y)
            v.append(state.v)
            t.append(state.t)

        plt.plot(x, y, 'y')
        plt.plot(v, t, 'r')

        plt.show()
        wait = input()
        plt.close()



def main(args=None):
    rclpy.init(args=args)
    trajectory_plotter = TrajectoryPlotter()
    rclpy.spin(trajectory_plotter)
    trajectory_plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print("Listening.....")
    main()