#!/bin/bash

# This file is to be used in the *root* of your Catkin workspace.

# This is a convenient script which will set up your ROS environment and
# should be executed with every new instance of a shell in which you plan on
# working with Intera.

# ----- EDIT THIS TO MATCH THE IP OF THE ROBOT
FRANKA_ROBOT_IP="FRANKA_ROBOT_IP.local"

ros_version="kinetic"


tf=$(mktemp)
trap "rm -f -- '${tf}'" EXIT

topdir=$(basename $(readlink -f $(dirname ${BASH_SOURCE[0]})))

cat <<-EOF > ${tf}
    [ -s "\${HOME}"/.bashrc ] && source "\${HOME}"/.bashrc
    [ -s "\${HOME}"/.bash_profile ] && source "\${HOME}"/.bash_profile

    # verify this script is moved out of franka_sdk folder
    if [[ -e "${topdir}/intera_sdk/package.xml" ]]; then
        echo -ne "EXITING - This script must be moved from the franka_sdk folder \
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
    # if set, verify user has modified the FRANKA_ROBOT_IP
    if [ -n ${FRANKA_ROBOT_IP} ] && \
    [[ "${FRANKA_ROBOT_IP}" == "FRANKA_ROBOT_IP.local" ]]; then
        echo -ne "EXITING - Please edit this file, modifying the \
'FRANKA_ROBOT_IP' variable to reflect your Robot's current ip address.\n"
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
    alias franka_untuck="roslaunch franka_example_controllers move_to_start.launch robot_ip:=$FRANKA_ROBOT_IP"


    # source the catkin setup bash script
    source devel/setup.bash

    # setup the bash prompt
    export __ROS_PROMPT=\${__ROS_PROMPT:-0}
    [ \${__ROS_PROMPT} -eq 0 -a -n "\${PROMPT_COMMAND}" ] && \
        export __ORIG_PROMPT_COMMAND=\${PROMPT_COMMAND}

    __ros_prompt () {
        if [ -n "\${__ORIG_PROMPT_COMMAND}" ]; then
            eval \${__ORIG_PROMPT_COMMAND}
        fi
        if ! echo \${PS1} | grep '\[franka' &>/dev/null; then
            export PS1="\[\033[00;33m\][franka - \
\${FRANKA_ROBOT_IP}]\[\033[00m\] \${PS1}"
        fi
    }

    if [ "\${TERM}" != "dumb" ]; then
        export PROMPT_COMMAND=__ros_prompt
        __ROS_PROMPT=1
    elif ! echo \${PS1} | grep '\[franka' &>/dev/null; then
        export PS1="[franka - \${FRANKA_ROBOT_IP}] \${PS1}"
    fi

EOF

${SHELL} --rcfile ${tf}

rm -f -- "${tf}"
trap - EXIT


