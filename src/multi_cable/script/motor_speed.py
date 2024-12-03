#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

class MotorSpeedController:
    def __init__(self):
        rospy.init_node('motor_speed_controller', anonymous=True)
        self.publisher = rospy.Publisher('~/iris/gazebo/command/motor_speed', Float64MultiArray, queue_size=10)
        self.current_speed = [5000, 5000, 5000, 5000]  # 默认速度
        rospy.loginfo("Motor Speed Controller Node has been started.")
        
        # 初始发布默认速度
        self.set_motor_speed(self.current_speed)
        
        # 设置循环频率
        self.rate = rospy.Rate(1000)  # 10 Hz
        self.run()

    def set_motor_speed(self, speed_values):
        msg = Float64MultiArray()
        msg.data = speed_values  # 设置旋翼的转速
        self.publisher.publish(msg)
        rospy.loginfo("Publishing motor speeds: %s", speed_values)

    def run(self):
        # 循环接受用户输入的速度
        while not rospy.is_shutdown():
            try:
                user_input = input("Enter motor speeds (4 values separated by spaces, e.g., '500 500 500 500') or press Enter to keep previous values: ")
                
                # 如果输入为空，则保持当前速度
                if user_input.strip() != "":
                    speed_values = [float(value) for value in user_input.split()]
                    
                    # 检查输入的值是否有4个，适用于四旋翼
                    if len(speed_values) == 4:
                        self.current_speed = speed_values
                    else:
                        rospy.logerr("Please enter exactly 4 speed values.")
                
                # 发布当前速度值（保持或更新后的值）
                self.set_motor_speed(self.current_speed)
                
            except ValueError:
                rospy.logerr("Invalid input. Please enter 4 numeric values.")
            
            # 等待下一次循环
            self.rate.sleep()

if __name__ == '__main__':
    try:
        MotorSpeedController()
    except rospy.ROSInterruptException:
        pass
