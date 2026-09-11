import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
from enum import  Enum

scanning_angle = 10.0
scanning_duration = 1.0
rotated_angle = 0
max_linear = 2.0
max_angular = 2.0
TIMEOUT = 5.0

#temporary conditions for boxes .CHANGE LATER 
containes_real = False 
containes_fake = False 
moving_forward_duration =1.0
moving_left_duration =1.0
localize_duration = 0.5


class Status(Enum):
    moveLeft = 1
    Rotate_90_1 = 2
    real_fake = 3
    real_only =4 # moving forward in each case 
    fake_only= 5  # moving forward in each case 
    move_forward_2 = 6
    Rotate_90_2 =  7
 

#timer block 
def countdown_timer(seconds):
    print("Timer started!")
    
    while seconds > 0:
        mins, secs = divmod(seconds, 60)
        # Wait for 1 second
        time.sleep(1)
        seconds -= 1
        
    return True

class Compete(Node):
    def __init__(self):
        super().__init__('compete')
        self.ultra_seen = False
        self.start_time = None
        self.box_seen = False
        self.d = None
        self.create_subscription(Float32, '/ultrasonic_distance', self.ultra_cb, 10)
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.target_class = "suitcase"  # set to your real target class
        self.real_class = "real_box"
        self.fake_class = "fake_box"
        self.create_subscription(Image, '/mono/image', self.image_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def ultra_cb(self, msg):
        self.ultra_seen = True
        self.d = msg.data

    def image_cb(self, msg):
        global containes_real, containes_fake
        if self.box_seen:
            return  # already found it, stop spending CPU on inference
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        results = self.model(cv_image, verbose=False)
        for box in results[0].boxes:
            name = results[0].names[int(box.cls[0])]
            conf = float(box.conf[0])
            if name == self.target_class and conf > 0.6:
                self.box_seen = True
            if name == self.real_class and conf > 0.6:
                containes_real = True
            if name == self.fake_class and conf > 0.6:
                containes_fake = True

    def wait_until(self, check_fn, timeout):
        """Spin the node while waiting for check_fn() to become True."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if check_fn():
                return True
        return False

    def rotate(self, degrees):
        """Positive degrees = turn left (CCW), negative = turn right (CW)."""
        angle_rad = math.radians(degrees)
        duration = abs(angle_rad) / max_angular

        cmd = Twist()
        cmd.angular.z = max_angular if degrees > 0 else -max_angular

        self.get_logger().info(f'Rotating {degrees} degrees ({duration:.2f}s)')
        start = time.monotonic()
        while time.monotonic() - start < duration:
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)

        self.cmd_vel_pub.publish(Twist()) 

    def compete(self):
        global rotated_angle, containes_real, containes_fake
        real = 0
        fake = 0
        d = self.d
        self.get_logger().info('Waiting for the robot to start...')
        if not self.wait_until(lambda: self.ultra_seen, TIMEOUT):
            self.get_logger().error('No /ultrasonic_distance -- robot did not start.')
            return False
        current_state = Status.moveLeft
        #moving left  - step1
        if current_state == Status.moveLeft :
         if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info("Moving left for 1 second...")

         if time.time() - self.start_time < moving_left_duration :
            # Under 1 second: Create and publish the left movement command
            move_left = Twist()
            move_left.linear.y = max_linear # Positive Y moves to the left
            self.cmd_vel_pub.publish(move_left)
         else:
            self.get_logger().info("moved left")
            self.cmd_vel_pub.publish(Twist())
            # Reset tracker and transition to a stop state
            self.start_time = None 

        # wait for localization time :           
        if self.start_time is None:
          self.start_time = time.time()
          self.get_logger().info(f"scanning for {localize_duration} second...")             
          if time.time() - self.start_time > localize_duration:
            # Reset tracker and transition to a stop state
            self.start_time = None 
        current_state = Status.Rotate_90_1


        #scaning - step 2 :
        if current_state  == Status.Rotate_90_1 : 
           while rotated_angle <=90.0 :
             count = 1
             self.get_logger().info(f"scanning .... moving{count} 10 degrees")
             rotated_angle +=10.0
             self.rotate(scanning_angle)
             self.get_logger().info(f"rotated {rotated_angle} degrees")
             
             self.cmd_vel_pub.publish(Twist())
             count +=1
             self.get_logger().info('Looking for box...')
                #  if not self.wait_until(lambda: self.box_seen, TIMEOUT):
                #self.get_logger().error('Box never detected.')
                #return False # will change later


            # turn on the model by spinning the node so image_cb runs and
            # updates containes_real / containes_fake
             rclpy.spin_once(self, timeout_sec=0.1)

            #scanning wait 
             if self.start_time is None:
                self.start_time = time.time()
                self.get_logger().info(f"scanning for {scanning_duration} second...")
                                        
                if time.time() - self.start_time > scanning_duration:
                    # Reset tracker and transition to a stop state
                    self.start_time = None 


              #checking      
             if containes_real == True and containes_fake == True: 
                self.get_logger().info("both boxes were deteced")
                current_state = Status.real_fake
             else :
                self.get_logger().info("scanning again")
             # after finishing the scan :
             #condition of seeing the boxes : 
           if containes_real == True and containes_fake == True: 
               self.get_logger().info("both boxes were deteced")
               current_state = Status.real_fake

           if containes_real == True and containes_fake == False: 
                self.get_logger().info("only the real box was detected")
                current_state = Status.real_only
                fake = 0
                real = 1
           if containes_real == False and containes_fake == True: 
                self.get_logger().info("only the fake box was detected")
                current_state = Status.fake_only
                fake = 1
                real = 0
           else : 
                self.get_logger().info("mission failed , no box was seen ")
            

            


        # moving forward 
        if current_state == Status.fake_only or current_state == Status.real_only :
            if d >= 10.0 :
              if self.start_time is None:
                self.start_time = time.time()
                self.get_logger().info(f"Moving forward for {moving_forward_duration} second...")
       
              if time.time() - self.start_time < moving_forward_duration:
                # Under 1 second: Create and publish the left movement command
                move_forward = Twist()
                move_forward.linear.x = max_linear # Positive x to move forward
                self.cmd_vel_pub.publish(move_forward)
              else:
                self.get_logger().info("moved forward")
                self.cmd_vel_pub.publish(Twist())
                # Reset tracker and transition to a stop state
                self.start_time = None 
             # wait for localization time : 
                                  
              if self.start_time is None:
                self.start_time = time.time()
                self.get_logger().info(f"scanning for {localize_duration} second...")             
                if time.time() - self.start_time > localize_duration:
                    # Reset tracker and transition to a stop state
                    self.start_time = None 
            else :
              self.cmd_vel_pub.publish(Twist())
             
        current_state = Status.Rotate_90_2


        #rotating again to look for the other box 

        if current_state  == Status.Rotate_90_2 : 
                self.get_logger().info(f"moving 90 degrees to face the boxes")
                self.rotate(-90.0)
                self.get_logger().info(f"rotated {rotated_angle} degrees")
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info('Looking for box...')
                        #  if not self.wait_until(lambda: self.box_seen, TIMEOUT):
                        #self.get_logger().error('Box never detected.')
                        #return False # will change later
                    # turn on the model by spinning the node so image_cb runs
                    # and updates containes_real / containes_fake
                rclpy.spin_once(self, timeout_sec=0.1)

                    # wait for scanning time : 

                if self.start_time is None:
                    self.start_time = time.time()
                    self.get_logger().info(f"scanning for {scanning_duration} second...")
                           
                    if time.time() - self.start_time > scanning_duration:
                        # Reset tracker and transition to a stop state
                        self.start_time = None 



                if containes_real == True and containes_fake == True: 
                  self.get_logger().info("MISSION COMPLETED ! , switching to manual ")
                  current_state = Status.real_fake

                elif containes_real == False and real == 0: # if the real box was deteced and it is still the only one detected again we must do something 
                    self.get_logger().info("the real box still couldn't be deteced")
                    current_state = Status.move_forward_2
        
                elif containes_fake == False and fake == 0: # if the fake box was deteced and it is still the only one detected again we must do something 
                    self.get_logger().info("the fake box still couldn't be deteced")
                    current_state = Status.move_forward_2
        
                elif containes_real == True and real == 0: # the other box got detected 
                    self.get_logger().info("the other box got detected , MISSION COMPLETED ! , switching to manual ")
                    current_state = Status.real_fake
                elif containes_fake == True and fake == 0: # the other box got detected 
                    self.get_logger().info("the other box got detected , MISSION COMPLETED !, switching to manual ")
                    current_state = Status.real_fake


        # final chance moving forward to find the hidden box 


        if current_state == Status.move_forward_2 :
            if d >= 10.0 :
                if self.start_time is None:
                 self.start_time = time.time()
                 self.get_logger().info(f"Moving forward for {moving_forward_duration} second...")
                  
                if time.time() - self.start_time < moving_forward_duration:
                  # Under 1 second: Create and publish the left movement command
                 move_forward = Twist()
                 move_forward.linear.x = max_linear # Positive x to move forward
                 self.cmd_vel_pub.publish(move_forward)
                else:
                 self.get_logger().info("moved forward")
                 self.cmd_vel_pub.publish(Twist())
                    # Reset tracker and transition to a stop state
                 self.start_time = None 
            else :
                self.cmd_vel_pub.publish(Twist())
                # turn on the model by spinning the node so image_cb runs
                # and updates containes_real / containes_fake
                rclpy.spin_once(self, timeout_sec=0.1)
            
        if containes_real == True and containes_fake == True: 
                self.get_logger().info("MISSION COMPLETED !")
                current_state = Status.real_fake
        elif containes_real == True and real == 0: # the other box got detected 
                self.get_logger().info("MISSION COMPLETED !")
                current_state = Status.real_fake
        elif containes_fake == True and fake == 0: # the other box got detected 
                self.get_logger().info("MISSION COMPLETED !")
                current_state = Status.real_fake
        else : 
                self.get_logger().info("mission failed , both boxes couldn't be found together ")
        
        if current_state == Status.real_fake :
            #manual code switching
            pass

        self.get_logger().info('Competition complete.')
        return True


def main():
    rclpy.init()
    node = Compete()
    node.compete()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
