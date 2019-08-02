import rospy
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt

vals = []

def callback(msg):

    global vals 
    vals = np.asarray(msg.position[:7]).tolist()
    # vals.append(msg.position[:7])
    # print vals



if __name__ == '__main__':
    

    rospy.init_node("test_node")
    pub = rospy.Publisher('/franka_control_ros/joint_position_commands',JointState, queue_size = 1, tcp_nodelay = True)
    rospy.Subscriber('/joint_states', JointState, callback)
    
    rate = rospy.Rate(200)
    upd = 0.002

    delta = upd

    max_val = 1.1897
    min_val = 0.4723473991867569

    print "Not recieved msg"

    while len(vals) != 7 and not rospy.is_shutdown():
        continue

    initial_pose = vals

    plt.ion()
    a = []
    i = 0.01
    while not rospy.is_shutdown():

        delta = 3.14 / 16 * (1 - np.cos(3.14 / 5.0 * i)) * 0.2

        # plt.plot(a)
        # plt.pause(0.00001)
        
        # if vals[6] >= max_val:
        #     delta = -upd
        # if vals[6] <= min_val:
        #     delta = upd


        pubmsg = JointState()
        pubmsg.position = vals
        pubmsg.position[6] = pubmsg.position[6] + delta
        print pubmsg.position[6]

        i+=0.01
        # # print pubmsg.position
        # # i+=1
        # # print vals
        pub.publish(pubmsg)
        rate.sleep()