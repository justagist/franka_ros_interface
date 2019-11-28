
import moveit_msgs.msg
import geometry_msgs.msg
import quaternion

def create_pose_stamped_msg(position, orientation, frame = "world"):

    pose = geometry_msgs.msg.PoseStamped()

    pose.header.frame_id = frame

    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]

    if isinstance(orientation,quaternion.quaternion):
        pose.pose.orientation.x = orientation.x
        pose.pose.orientation.y = orientation.y
        pose.pose.orientation.z = orientation.z
        pose.pose.orientation.w = orientation.w
    else:
        pose.pose.orientation.x = orientation[1]
        pose.pose.orientation.y = orientation[2]
        pose.pose.orientation.z = orientation[3]
        pose.pose.orientation.w = orientation[0]

    return pose
    