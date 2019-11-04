
import numpy as np
from franka_control.srv import SetEEFrame
import rospy

DEFAULT_EE_FRAME = [0.707099974155426, -0.707099974155426, 0.0, 0.0, 0.707099974155426, 0.707099974155426, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.10339999943971634, 1.0] # default when the franka_ros control is launched

class FrankaFramesInterface():
    """
        Helper class to retrieve and set EE frames, [and K frame (not implemented)]

    """

    def __init__(self, robot):

        self._robot = robot
        self._current_EE_frame_transformation = self._robot.get_current_EE_frame_transformation(as_mat = True) # transformation matrix defining the current EE frame with respect to the flange frame


        rospy.wait_for_service('/franka_control/set_EE_frame')
        self._EE_frame_request_handler = rospy.ServiceProxy('/franka_control/set_EE_frame', SetEEFrame)


    def set_EE_frame(self, frame):
        """
        Set new EE frame based on the transformation given by 'frame', which is the 
        transformation matrix defining the new desired EE frame with respect to the flange frame.

        @type frame: [float (16,)] / np.ndarray (4x4) 
        @param frame: transformation matrix of new EE frame wrt flange frame
        @rtype: bool
        @return: success status of service request
        """
        if isinstance(frame, np.ndarray):
            if frame.shape[0] == frame.shape[1] == 4:
                frame = frame.flatten('F').tolist()
            else:
                raise ValueError("Invalid shape for transformation matrix numpy array")
        else:
            assert len(frame) == 16, "Invalid number of elements in transformation matrix. Should have 16 elements."

        self._request_setEE_service(frame)

        pass

    def get_EE_frame(self, as_mat = False):
        """
        Get current EE frame transformation matrix in flange frame
        
        @type as_mat: bool
        @param as_mat: if True, return np array, else as list
        @rtype: [float (16,)] / np.ndarray (4x4) 
        @return: transformation matrix of EE frame wrt flange frame
        """
        return self._robot.get_current_EE_frame_transformation(as_mat = as_mat)

    def reset_EE_frame(self):
        """
        Reset EE frame to default. (defined by DEFAULT_EE_FRAME global variable defined above) 

        @rtype: bool
        @return: success status of service request
        """

        pass

    def _request_setEE_service(self, trans_mat):

        try:
            response = self._EE_frame_request_handler(F_T_EE = trans_mat)
            rospy.loginfo("Set EE Frame Request Status: %s. \n\tDetails: %s"%("Success" if response.success else "Failed!", response.error))
        except rospy.ServiceException, e:
            rospy.logwarn("Activation service call failed: %s"%e)
            return False


    def set_K_frame(self, frame):
        """
        Set new K frame based on the transformation given by 'frame', which is the 
        transformation matrix defining the new desired K frame with respect to the EE frame.

        @type frame: [float (16,)] / np.ndarray (4x4) 
        @param frame: transformation matrix of new K frame wrt EE frame
        @rtype: bool
        @return: success status of service request
        """
        raise NotImplementedError("Not defined yet")
        

    def get_K_frame(self, as_mat = False):
        """
        Get current K frame transformation matrix in EE frame
        
        @type as_mat: bool
        @param as_mat: if True, return np array, else as list
        @rtype: [float (16,)] / np.ndarray (4x4) 
        @return: transformation matrix of K frame wrt EE frame
        """
        raise NotImplementedError("Not defined yet")

    def reset_K_frame(self):
        """
        Reset K frame to default. (defined by DEFAULT_FRAME global variable defined above) 

        @rtype: bool
        @return: success status of service request
        """

        raise NotImplementedError("Not defined yet")


if __name__ == '__main__':
    # main()
    from franka_interface import ArmInterface

    rospy.init_node("test")

    ee_setter = FrankaFramesInterface(ArmInterface())

    ee_setter.set_EE_frame([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1])

    
