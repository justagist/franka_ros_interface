#!/bin/bash

# Copyright (c) 2019-2021, Saif Sidhik
# Copyright (c) 2013-2016, Rethink Robotics
# All rights reserved.

# This is a convenient script which will set up your ROS environment and
# should be executed with every new instance of a shell in which you plan on
# working with Franka ROS Interface or PandaRobot.

# This file is to be used in the *root* of your catkin workspace.

# ****** IMPORTANT: Read "USAGE EXPLAINED" section below before running this script. ******



# Run as './franka.sh <arg1> <arg2>'

# ------ ARGUMENTS:
# -- arg1 (required): select the environment for Franka ROS Interface. The available options are:
#        - 'master' / 'local' (No <arg2>): This can be used only if this machine can succesfully connect to the robot using libfranka. By setting this, this machine will be assumed to be the ros master. The 'driver' node can only be run in this environment (see Usage instructions in readme).
#        - 'slave' / 'remote': This argument should be used if the current machine cannot directly to franka controller, but is in the same network as the machine that is running the 'master' (and the 'driver' nodes). If IP address is provided as <arg2>, this will connected to a ROS Master at that IP. If <arg2> is not provided, ROS_MASTER_IP variable set below will be used.
#        - 'sim': start in simulation environment (to be used with panda_simulator package only). [Not necessary]

# -- arg2 (optional):
#        - <IP of ROS MASTER> (valid only if <arg1> is 'slave'/'remote'): IP address of ROS Master

# eg: 
#   ./franka.sh master
#   (or)
#   ./franka.sh remote [optional IP address of master]


## ========================================= #
## === EDIT FOLLOWING VALUES AS REQUIRED === #
## ========================================= #

# ----- EDIT THIS TO MATCH THE IP OF THE FRANKA ROBOT CONTROLLER IN THE NETWORK
FRANKA_ROBOT_IP="FRANKA_ROBOT_IP.local"

# ----- EDIT THIS TO MATCH YOUR IP IN THE NETWORK
your_ip=""

# ----- EDIT THIS TO MATCH THE IP OF THE MAIN ROS MASTER PC (CONNECTED TO THE FRANKA CONTROLLER),
# ----- ON WHICH THE Franka ROS Interface 'driver' NODES WILL BE RUNNING (SEE "USAGE EXPLAINED" BELOW).
# ----- THIS VALUE IS NEEDED ONLY IF YOU WILL USE THE 'remote' ENVIRONMENT ON THIS MACHINE.
ROS_MASTER_IP=""

ros_version="melodic"

## ========================================= #
## ========== USAGE EXPLAINED ============== #
## ========================================= #

# 1. If there is only one computer controlling the robot, there is no need 
#    to fill the $ROS_MASTER_IP variable in the field above. However, this
#    machine should have RT kernel set up correctly and should be able to 
#    connect to the robot using libfranka. 
#    For using Franka ROS Interface in this case:
#       a) Move this file to the root of the catkin ws, and fill in the 
#          values for $FRANKA_ROBOT_IP and $your_ip above.
#       b) run `./franka.sh master`
#       c) Start the driver node (see Usage in README)
#       d) Once the driver node is running correctly, on a different 
#          terminal, run `./franka master` again from the catkin_ws root to 
#          connect to the same ROS master as the driver node.
#       e) You should now be able to communicate with the robot using 
#          Franka ROS Interface from this terminal session. Use ROS 
#          topics/services etc. or Franka ROS Interface Python API or scripts 
#          to control and monitor the robot. (test by running `rostopic list`)
#       f) For every new terminal, make sure to run `./franka.sh master` first.

# 2. If you want to use multiple machines for controlling the robot, all 
#    machines should be in the same network as the robot controller.
#    AT LEAST ONE MACHINE should have RT kernel set up and should be able to 
#    properly connect to the robot using libfranka. ONE of these machines 
#    should run the driver node as explained in steps 1.a - 1.c above. 
#    Follow steps 1.d - 1.f if Franka ROS Interface is needed againg on this 
#    RT machine (which is running the driver).
#
#    NOTE: ONLY ONE MACHINE IN THE NETWORK SHOULD BE RUNNING THE DRIVER NODE.
#
#    Other computers in the network need not have RT kernels, but should have 
#    correctly installed lifranka, franka-ros, and Franka ROS Interface. For 
#    controlling the robot from these machines:
#       a) Move this file to the root of the catkin_ws of the remote machines, 
#          and fill in the values for $FRANKA_ROBOT_IP, $your_ip, and 
#          $ROS_MASTER_IP above. THE $ROS_MASTER_IP should correspond to the 
#          IP address of the RT machine running the 'driver' node.
#       b) Then run `./franka.sh remote`. This should connect the current 
#          session to the ROS master running the driver node (to test if the 
#          connection is correct, run `rostopic list` to see if the published 
#          and subscribed topics from Franka ROS Interface driver node is 
#          accessible from the remote computer).
#       c) As long as the driver node is running on the RT machine, you should 
#          now be able to communicate with the robot using Franka ROS Interface 
#          from this terminal session. Use ROS topics/services etc. or Franka 
#          ROS Interface Python API or scripts to control and monitor the robot.
#       d) For every new terminal in the remote machines, make sure to run 
#          `./franka.sh remote` first.


## ========================================= #
## ========================================= #
## === DO NOT EDIT BELOW THIS LINE ========= #


if [ "${1}" == "sim" ]; then
    your_ip="127.0.0.1"
fi

if [ "${your_ip}" == "" ]; then
    echo -e "\n\t\033[01;31m\033[1mError:\033[00m [franka.sh] \033[01;33m\033[4myour_ip\033[00m variable has to be set in 'franka.sh'\n"
    exit 1
fi



if [ "${1}" == "master" ] || [[ "${1}" == "local" ]]; then
    if [ "${FRANKA_ROBOT_IP}" == "FRANKA_ROBOT_IP.local" ]; then
    echo -e "\n\t\033[01;31m\033[1mError:\033[00m [franka.sh] \033[01;33m\033[4mFRANKA_ROBOT_IP\033[00m variable has to be set to match the IP address of the Franka Robot Controller\n"
    exit 1
    fi
    ROS_MASTER_IP="$your_ip"
    IS_MASTER=true
elif [ "${1}" == "slave" ] || [[ "${1}" == "remote" ]]; then
    if [[ $# -ge 2 ]]; then
        ROS_MASTER_IP="$2"
    fi
    IS_MASTER=false  
elif [ "${1}" == "sim" ]; then
    ROS_MASTER_IP="$your_ip"
    IS_MASTER="true"
    FRANKA_ROBOT_IP="sim"
else
    echo -e "\n\t\033[01;31m\033[1mError:\033[00m [franka.sh] Atleast one argument has to be passed
        
        Usage: ./franka_sdk <arg1> <arg2>

        ------ ARGUMENTS:
        -- arg1:
               - 'master' / 'local' (No <arg2>): The local machine will be the ros master. Run this on the main master pc (connected to the franka controller). 
               - 'slave' / 'remote': If IP address is provided as <arg2>, this will make sure that this PC is connected to the ROS Master at that IP. If <arg2> provided, ROS_MASTER_IP set below will be used
               - 'sim': Franka robot simulation environment will be used. Real robot will not be controlled.

        -- arg2:
               - <IP of ROS MASTER> (valid only if <arg1> is 'slave'/'remote'): IP address of ROS Master
           "
   exit 1
fi

tf=$(mktemp)
trap "rm -f -- '${tf}'" EXIT

topdir=$(basename $(readlink -f $(dirname ${BASH_SOURCE[0]})))

cat <<-EOF > ${tf}
    [ -s "\${HOME}"/.bashrc ] && source "\${HOME}"/.bashrc
    [ -s "\${HOME}"/.bash_profile ] && source "\${HOME}"/.bash_profile

    # verify this script is moved out of franka_sdk folder
    if [[ -e "${topdir}/franka_rospy_sdk/package.xml" ]]; then
        echo -ne "EXITING - This script must be moved from the franka_rospy_sdk folder \
to the root of your catkin workspace.\n"
        exit 1
    fi


    # check for ros installation
    if [ ! -d "/opt/ros" ] || [ ! "$(ls -A /opt/ros)" ]; then
        echo "EXITING - No ROS installation found in /opt/ros."
        echo "Is ROS installed?"
        exit 1
    fi

    # verify specified ros version is installed
    ros_setup="/opt/ros/${ros_version}"
    if [ ! -d "\${ros_setup}" ]; then
        echo -ne "EXITING - Failed to find ROS \${ros_version} installation \
in \${ros_setup}.\n"
        exit 1
    fi

    # verify the ros setup.sh file exists
    if [ ! -s "\${ros_setup}"/setup.sh ]; then
        echo -ne "EXITING - Failed to find the ROS environment script: \
"\${ros_setup}"/setup.sh.\n"
        exit 1
    fi

    # verify the user is running this script in the root of the catkin
    # workspace and that the workspace has been compiled.
    if [ ! -s "devel/setup.bash" ]; then
        echo -ne "EXITING - \n1) Please verify that this script is being run \
in the root of your catkin workspace.\n2) Please verify that your workspace \
has been built (source /opt/ros/\${ros_version}/setup.sh; catkin_make).\n\
3) Run this script again upon completion of your workspace build.\n"
        exit 1
    fi

    export FRANKA_ROBOT_IP=$FRANKA_ROBOT_IP

    [ -n "${your_ip}" ] && export ROS_IP="${your_ip}"
    [ -n "${ROS_MASTER_IP}" ] && \
        export ROS_MASTER_URI="http://${ROS_MASTER_IP}:11311"


    # source the catkin setup bash script
    source devel/setup.bash


    if [[ $IS_MASTER == true ]]; then
        echo -e "\nROBOT: $FRANKA_ROBOT_IP\n"
    else
        echo -e "\nROS MASTER: $ROS_MASTER_IP\n"    
    fi

    # setup the bash prompt
    export __ROS_PROMPT=\${__ROS_PROMPT:-0}
    [ \${__ROS_PROMPT} -eq 0 -a -n "\${PROMPT_COMMAND}" ] && \
        export __ORIG_PROMPT_COMMAND=\${PROMPT_COMMAND}

    __ros_prompt () {
        if [ -n "\${__ORIG_PROMPT_COMMAND}" ]; then
            eval \${__ORIG_PROMPT_COMMAND}
        fi
        if ! echo \${PS1} | grep '\[franka' &>/dev/null; then
            if [[ $your_ip == "localhost" ]]; then
                export PS1="\[\033[00;33m\][franka <SIM> - Robot@\
${FRANKA_ROBOT_IP}]\[\033[00m\] \${PS1}"
            elif [[ $IS_MASTER == true ]]; then
                export PS1="\[\033[00;33m\][franka <Master> - Robot@\
${FRANKA_ROBOT_IP}]\[\033[00m\] \${PS1}"
            else
                export PS1="\[\033[00;33m\][franka <Remote> - Master@\
${ROS_MASTER_IP}]\[\033[00m\] \${PS1}"
            fi
        fi
    }

    if [ "\${TERM}" != "dumb" ]; then
        export PROMPT_COMMAND=__ros_prompt
        __ROS_PROMPT=1
    elif ! echo \${PS1} | grep '\[franka' &>/dev/null; then
        if [[ $your_ip == "localhost" ]]; then
            export PS1="\[\033[00;33m\][franka <SIM> - Robot@\
${FRANKA_ROBOT_IP}]\[\033[00m\] \${PS1}"
        elif [[ $IS_MASTER == true ]]; then
            export PS1="\[\033[00;33m\][franka <Master> - Robot@\
${FRANKA_ROBOT_IP}]\[\033[00m\] \${PS1}"
        else
            export PS1="\[\033[00;33m\][franka <Remote> - Master@\
${ROS_MASTER_IP}]\[\033[00m\] \${PS1}"
        fi
    fi

EOF

${SHELL} --rcfile ${tf}

rm -f -- "${tf}"
trap - EXIT
