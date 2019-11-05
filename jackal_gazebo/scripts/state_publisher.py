#!/usr/bin/env python
# import rospy  
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64


# class CommandToJointState:
#     def __init__(self):
#         # self.joint_name = rospy.get_param("base_link")
#         self.joint_state = JointState()
#         # self.joint_state.name.append(self.joint_name)
#         self.joint_state.position.append(0.0)
#         self.joint_state.velocity.append(0.0)
#         self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
#         self.command_sub = rospy.Subscriber("base_link", Float64,
#                                             self.command_callback, queue_size=10)

#     def command_callback(self, msg):
#         self.joint_state.position[0] = [5,5,1]
#         self.joint_state.header.stamp = rospy.Time.now()
#         self.joint_pub.publish(self.joint_state)

# if __name__ == '__main__':
#     rospy.init_node('Fused_joint_state')
#     command_to_joint_state = CommandToJointState()
#     rospy.spin()


import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('fused_state_publisher')
    rate = rospy.Rate(500) # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel','rear_right_wheel']
    hello_str.position = [0, 0, 0, 0]
    hello_str.velocity = []
    hello_str.effort = []
    pub.publish(hello_str)
    rate.sleep()

    while not rospy.is_shutdown():
        hello_str.header.stamp = rospy.Time.now()
        pub.publish(hello_str)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass