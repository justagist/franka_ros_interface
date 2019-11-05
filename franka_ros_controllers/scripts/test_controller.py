import rospy
from sensor_msgs.msg import JointState
from franka_core_msgs.msg import JointCommand
import numpy as np
import matplotlib.pyplot as plt

vals = []
vels = []
def callback(msg):

    global vals , vels
    vals = np.asarray(msg.position[:7]).tolist()
    vels = np.asarray(msg.velocity[:7]).tolist()
    # vals.append(msg.position[:7])
    # print vals
  # elapsed_time_ += period;

  # double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;
  # for (size_t i = 0; i < 7; ++i) {
  #   if (i == 4) {
  #     position_joint_handles_[i].setCommand(initial_pose_[i] - delta_angle);
  #   } else {
  #     position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle);
  #   }
  # }


if __name__ == '__main__':
    

    rospy.init_node("test_node")
    pub = rospy.Publisher('/franka_ros_interface/effort_joint_impedance_controller/joint_commands',JointCommand, queue_size = 1, tcp_nodelay = True)
    # pub = rospy.Publisher('/franka_ros_interface/position_joint_position_controller/joint_commands',JointCommand, queue_size = 1, tcp_nodelay = True)
    rospy.Subscriber('/franka_ros_interface/custom_franka_state_controller/joint_states', JointState, callback)
    
    rate = rospy.Rate(100)
    upd = 0.002

    delta = upd

    max_val = 1.1897
    min_val = 0.4723473991867569

    print "Not recieved msg"

    while len(vals) != 7 and not rospy.is_shutdown():
        continue

    initial_pose = vals
    # plt.ion()
    a = []
    i = 0.1


    elapsed_time_ = rospy.Duration(0.0)
    period = rospy.Duration(0.005)

    while not rospy.is_shutdown():

        elapsed_time_ += period

        delta = 3.14 / 16.0 * (1 - np.cos(3.14 / 5.0 * elapsed_time_.to_sec())) * 0.2

        for j in range(len(vals)):
            if j == 4:
                vals[j] = initial_pose[j] - delta
            else:
                vals[j] = initial_pose[j] + delta

        # plt.plot(a)
        # plt.pause(0.00001)
        
        # if vals[6] >= max_val:
        #     delta = -upd
        # if vals[6] <= min_val:
        #     delta = upd


        pubmsg = JointCommand()
        pubmsg.mode = pubmsg.TORQUE_MODE
        # pubmsg.mode = pubmsg.POSITION_MODE
        pubmsg.position = vals
        pubmsg.velocity = vels
        # pubmsg.position[6] = pubmsg.position[6] + delta
        # print pubmsg.position[6]

        i+=0.5
        # # print pubmsg.position
        # # i+=1
        # # print vals
        pub.publish(pubmsg)
        rate.sleep()