import rospy
from sensor_msgs.msg import JointState
from franka_core_msgs.msg import JointCommand
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from copy import deepcopy

vals = []
vels = []
names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

def callback(msg):

    global vals , vels
    dic = {}

    # for n in range(len(msg.name)):
    #     dic[msg.name[n]] = msg.position[n]
    temp_vals = []
    temp_vels = []
    for n in names:
        idx = msg.name.index(n)
        temp_vals.append(msg.position[idx])
        temp_vels.append(msg.velocity[idx])

    vals = deepcopy(temp_vals)
    vels = deepcopy(temp_vels)
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
    pub = rospy.Publisher('/franka_ros_interface/motion_controller/arm/joint_commands',JointCommand, queue_size = 1, tcp_nodelay = True)
    # pub = rospy.Publisher('/franka_ros_interface/position_joint_position_controller/arm/joint_commands',JointCommand, queue_size = 1, tcp_nodelay = True)
    # pub = rospy.Publisher('/panda_simulator/arm/joint_command',JointCommand, queue_size = 1, tcp_nodelay = True)
    # rospy.Subscriber('/panda_simulator/joint_states', JointState, callback)
    rospy.Subscriber('/franka_ros_interface/custom_franka_state_controller/joint_states', JointState, callback)
    
    # pub2 = rospy.Publisher("/panda_simulator/effort_joint_position_controller/joints/panda_joint6_controller/command", Float64)

    rate = rospy.Rate(1000)

    max_val = 1.1897
    min_val = 0.4723473991867569

    print "Not recieved msg"

    while len(vals) != 7 and not rospy.is_shutdown():
        continue

    initial_pose = vals
    # plt.ion()
    a = []
    i = 0.1


    print "commanding"
    elapsed_time_ = rospy.Duration(0.0)
    period = rospy.Duration(0.005)

    count = 0
    pubmsg = JointCommand()
    pubmsg.names = names # names of joints (has to be 7 and in the same order as the command fields (positions, velocities, efforts))
    while not rospy.is_shutdown():

        elapsed_time_ += period

        delta = 3.14 / 16.0 * (1 - np.cos(3.14 / 5.0 * elapsed_time_.to_sec())) * 0.2
        # delta = 0.001
        delta = 0.3*delta
        # if count%100 == 0:
        for j in range(len(vals)):
            if j == 4:
                vals[j] = initial_pose[j] - delta
            else:
                vals[j] = initial_pose[j] + delta

        # plt.plot(a)
        # plt.pause(0.00001)
        if count%500 == 0:
            print "\n ----  \n"
            print vals, delta
            print " "
            print initial_pose
        
        pubmsg.position = vals # JointCommand msg has other fields (velocities, efforts) for
                               # when controlling in other control mode
        pubmsg.velocities = vels
        pubmsg.mode = pubmsg.IMPEDANCE_MODE # Specify control mode (POSITION_MODE, VELOCITY_MODE, IMPEDANCE_MODE (not available in sim), TORQUE_MODE)

        pub.publish(pubmsg)
        rate.sleep()