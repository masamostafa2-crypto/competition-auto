import rclpy
from rclpy.node import Node
from pynput import keyboard

from geometry_msgs.msg import Twist


class ManualController(Node):

    def __init__(self):
        super().__init__('manual_controller')


        # Parameters

        self.declare_parameter('speed_mode', '2')


        self.current_mode = self.get_parameter(
            'speed_mode'
        ).value

        

        self.speed_modes = {
              '1': (0.15, 0.5),
              '2': (0.3, 1.0),
              '3': (0.6, 1.5)
             }


        self.linear_speed = self.speed_modes[self.current_mode][0]
        self.angular_speed = self.speed_modes[self.current_mode][1]

    
        self.linear = 0.0
        self.angular = 0.0

        
        # Publisher

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.keys_pressed = set()
        self.emergency_stop = False

         # Start keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
            )

        self.listener.start()

        self.get_logger().info(
            'Manual controller is available'
        )



    def on_press(self, key):
        # Emergency stop
        if key == keyboard.Key.space:
             self.emergency_stop = True
             self.linear = 0.0
             self.angular = 0.0
             self.get_logger().warn('EMERGENCY STOP')
             return

        # Reset emergency stop
        if key == keyboard.KeyCode.from_char('r'):
            self.emergency_stop = False
            self.get_logger().info('Emergency stop released')
            return

         # Don't accept movement commands during emergency stop
        if self.emergency_stop:
            return

        
        # Speed modes
        if key == keyboard.KeyCode.from_char('1'):
            self.set_speed_mode('1')
            return
        elif key == keyboard.KeyCode.from_char('2'):
            self.set_speed_mode('2')
            return

        elif key == keyboard.KeyCode.from_char('3'):
            self.set_speed_mode('3')
            return

        
        # Add the pressed key to the set of currently pressed keys
        self.keys_pressed.add(key)

        # Update movement based on the currently pressed keys
        self.update_movement()




    def on_release(self, key):

        if key in self.keys_pressed:
            self.keys_pressed.remove(key)
            self.update_movement()

    def update_movement(self):
        # Ensuring the robot doesn't move during an emergency stop
        if self.emergency_stop:
            self.linear = 0.0
            self.angular = 0.0
            return

        # Linear movement
        if keyboard.KeyCode.from_char('w') in self.keys_pressed:
            self.linear = self.linear_speed

        elif keyboard.KeyCode.from_char('s') in self.keys_pressed:
            self.linear = -self.linear_speed

        else:
            self.linear = 0.0

        # Angular movement
        if keyboard.KeyCode.from_char('a') in self.keys_pressed:
            self.angular = self.angular_speed

        elif keyboard.KeyCode.from_char('d') in self.keys_pressed:
            self.angular = -self.angular_speed

        else:
            self.angular = 0.0

    def set_speed_mode(self, mode):
        self.current_mode = mode

        self.linear_speed = self.speed_modes[mode][0]
        self.angular_speed = self.speed_modes[mode][1]

        self.get_logger().info(
           f'Speed mode {mode}: '
           f'linear={self.linear_speed}, '
           f'angular={self.angular_speed}'
         )



    def control_loop(self):

        msg = Twist()

        if self.emergency_stop:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        else:
            msg.linear.x = self.linear
            msg.angular.z = self.angular

        self.cmd_vel_pub.publish(msg)
   

    def destroy_node(self):

        # Stop keyboard listener
        self.listener.stop()

        # Stop the robot before shutting down
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

        super().destroy_node()





def main(args=None):

    rclpy.init(args=args)

    node = ManualController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
