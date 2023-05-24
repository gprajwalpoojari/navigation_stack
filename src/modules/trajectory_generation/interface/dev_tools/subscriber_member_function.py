import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from common_ros2.msg import Posture, Spline 
import matplotlib.pyplot as plt


# remaining
# add rostopic
# debug

class TrajectoryPlotter(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Spline,
            '/trajectory',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        plt.figure(figsize=(8,3))
        print(len(msg.spline_points))
        for state in msg.spline_points:
            # print(state.x,state.y)
            plt.plot(state.x, state.y, 'ro')

        plt.show()
        wait = input()
        plt.close()



def main(args=None):
    rclpy.init(args=args)

    trajectory_plotter = TrajectoryPlotter()

    rclpy.spin(trajectory_plotter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajectory_plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print("Listening.....")
    main()