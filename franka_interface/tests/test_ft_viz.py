# from franka_interface import 
import numpy as np
from geometry_msgs.msg import WrenchStamped, PoseStamped, Pose
from franka_core_msgs.msg import EndPointState
import rospy
import quaternion
from aml_robot.panda_robot import PandaArm

def change_K_frame(frame):
    frame = r.get_frames_interface()._assert_frame_validity(frame)

    active_controllers = r.get_controller_manager().list_active_controllers(only_motion_controllers = True)
    rospy.sleep(1.)
    rospy.loginfo("ArmInterface: Stopping motion controllers for changing EE frame")
    for ctrlr in active_controllers:
        r.get_controller_manager().stop_controller(ctrlr.name)
    rospy.sleep(1.)

    retval = r.get_frames_interface().set_K_frame(frame)

    rospy.sleep(1.)
    rospy.loginfo("ArmInterface: Restarting previously active motion controllers.")
    if len(active_controllers)>0:
        for ctrlr in active_controllers:
            r.get_controller_manager().start_controller(ctrlr.name)
        rospy.sleep(1.)

    return retval

def create_pose_msg(position, orientation):
    """
        Create Pose message using the provided position and orientation

        :return: Pose message for the give end-effector position and orientation
        :rtype: geometry_msgs.msg.Pose
        :param position: End-effector position in base frame of the robot
        :type position: [float]*3
        :param orientation: orientation quaternion of end-effector in base frame
        :type orientation: quaternion.quaternion (OR) [float] size 4: (w,x,y,z)
    """
    pose = Pose()

    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]

    if isinstance(orientation,quaternion.quaternion):
        pose.orientation.x = orientation.x
        pose.orientation.y = orientation.y
        pose.orientation.z = orientation.z
        pose.orientation.w = orientation.w
    else:
        pose.orientation.x = orientation[1]
        pose.orientation.y = orientation[2]
        pose.orientation.z = orientation[3]
        pose.orientation.w = orientation[0]

    return pose

def create_pose_stamped_msg(position, orientation, frame = "world"):
    """
        Create PoseStamped message using the provided position and orientation

        :return: Pose message for the give end-effector position and orientation
        :rtype: geometry_msgs.msg.Pose
        :param position: End-effector position in base frame of the robot
        :type position: [float]*3
        :param orientation: orientation quaternion of end-effector in base frame
        :type orientation: quaternion.quaternion (OR) [float] size 4: (w,x,y,z)
        :param frame: Name of the parent frame
        :type frame: str
    """
    pose = PoseStamped()

    pose.header.frame_id = frame

    pose.pose = create_pose_msg(position, orientation)

    return pose

def pub_msg(msg):
    pub.publish(msg.K_F_ext_hat_K)
    pub2.publish(msg.O_F_ext_hat_K)
    pub3.publish(pose1)
    pub4.publish(pose2)

if __name__ == "__main__":
    
    rospy.init_node("test_ft")
    pose1 = create_pose_stamped_msg(np.asarray([0,1,0]),quaternion.quaternion(-0.00181994870195759, -0.930293691604238, 0.365737058019154, 0.0280488776883357))
    pose2 = create_pose_stamped_msg(np.asarray([0,1,1]),quaternion.quaternion(0.00905397184633361, -0.999440751171704, -0.018109674367698, 0.0266129702484029))
    pub = rospy.Publisher("/ft_for_rviz", WrenchStamped, queue_size=1)
    pub2 = rospy.Publisher("/ft_for_rviz2", WrenchStamped, queue_size=1)
    pub3 = rospy.Publisher("/pose_for_rviz", PoseStamped, queue_size=1)
    pub4 = rospy.Publisher("/pose_for_rviz2", PoseStamped, queue_size=1)

    rospy.Subscriber("/franka_ros_interface/custom_franka_state_controller/tip_state",EndPointState, pub_msg)
    r = PandaArm(reset_frames=True)
    # r.set_EE_frame_to_link('panda_link8')

    ang = -np.pi/4
    rot_mat = np.array([[np.cos(ang), -np.sin(ang), 0,0],
                        [np.sin(ang), np.cos(ang), 0,0],
                        [0,0,1,0],
                        [0,0,0,1]]
                        )
    # r.set_EE_frame(rot_mat)
    # change_K_frame(rot_mat)

    # rospy.spin()


