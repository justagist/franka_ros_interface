#!/bin/bash

# Copyright (c) 2019, Saif Sidhik
# Copyright (c) 2013-2016, Rethink Robotics
# All rights reserved.


# This file is to be used in the *root* of your Catkin workspace.

# This is a convenient script which will set up your ROS environment and
# should be executed with every new instance of a shell in which you plan on
# working with Intera.

# Run as './franka.sh <arg1> <arg2>'

# ------ ARGUMENTS:
# -- arg1:
#        - 'master' / 'local' (No <arg2>): The local machine will be the ros master. Run this on the main master pc (connected to the franka controller). 
#        - 'slave' / 'remote': If IP address is provided as <arg2>, this will make sure that this PC is connected to the ROS Master at that IP. If <arg2> provided, ROS_MASTER_IP set below will be used
#        - 'sim': start in simulation environment (to be used with panda_simulator package only)

# -- arg2:
#        - <IP of ROS MASTER> (valid only if <arg1> is 'slave'/'remote'): IP address of ROS Master



## ========================================= #
## ===== EDIT THESE VALUES AS REQUIRED ===== #
## ========================================= #


# ----- EDIT THIS TO MATCH THE IP OF THE FRANKA ROBOT CONTROLLER IN THE NETWORK
FRANKA_ROBOT_IP="FRANKA_ROBOT_IP.local"

# ----- EDIT THIS TO MATCH YOUR IP IN THE NETWORK
your_ip=""

# ----- EDIT THIS TO MATCH THE IP OF THE MAIN ROS MASTER PC (CONNECTED TO THE FRANKA CONTROLLER)
ROS_MASTER_IP=""

ros_version="kinetic"

## ========================================= #
## ========================================= #
## ========================================= #

if [ "${1}" == "sim" ]; then
    your_ip="localhost"
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
    ROS_MASTER_IP=""
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
