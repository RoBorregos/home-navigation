import rclpy
from rclpy.node import Node


class Stm32ROS(Node):

    def __init__(self):
        super().__init__('DashgoDriver')
        self.declare_parameter('my_parameter', 'world')
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        # my_new_param = rclpy.parameter.Parameter(
        #     'my_parameter',
        #     rclpy.Parameter.Type.STRING,
        #     'world'
        # )
        # all_new_parameters = [my_new_param]
        # self.set_parameters(all_new_parameters) # set new parameters



def main():
    rclpy.init()

    Stm32Node = Stm32ROS()

    rclpy.spin(Stm32Node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


        # self._action_server = ActionServer(
        #     self,
        #     Fibonacci,
        #     'fibonacci',
        #     self.execute_callback)

    # def execute_callback(self, goal_handle):
    #     self.get_logger().info('Executing goal...')
    #     result = Fibonacci.Result()
    #     return result