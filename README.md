
# Petri Net Plans

This repository contains Petri Net Plans library, PNP ROS bridge and some sample applications. This master branch contains up-to-date development, with the indigo version of PNPros.

## How to install and run

PNP requires to use `catkin_make_isolated` which results in a slightly different layout of the `build` and `devel` directories in your ros workspace. Hence, I suggest to keep this in a separate workspace for cleaner results.

Possible workspace structure:

```
/opt/ros -> pnp_ws -> ros_ws
```

In the following, I will assume that you are aiming to achieve this structure on your system.

### Installation

1. Create a new workspace: `mkdir ~/pnp_ws/src`
2. Change into the workspace: `cd ~/pnp_ws/src`
3. Create a catkin workspace: `ctakin_init_workspace`
4. Clone the repository: `git clone git@protolab.aldebaran.com:mummer/petri_net_plans.git`
5. Make sure that only the installe ROS is sourced: `source /opt/ros/indigo/setup.bash`
6. Download dependencies: `rosdep install --from-paths petri_net_plans -y -r -n`
7. Install additional dependencies: `sudo apt-get install ros-indigo-occupancy-grid-utils ros-indigo-diagnostic-msgs`
8. Build the workspace: `catkin_make_isolated -C ~/pnp_ws/`

### Set-up your ROS environment

Assuming you now have a `pnp_ws` and you also have a `ros_ws`. Make sure that your `~/.bashrc` contains the following entries in the bottom:

```
source /opt/ros/indigo/setup.bash
source ~/pnp_ws/devel_isolated/setup.bash
source ~/ros_ws/devel/setup.bash
```

Afterwards, you will have to recompile your `~/ros_ws/` to make sure that you have a clean workspace overlay:

```
cd ~/ros_ws
rm -rf build devel
source ~/pnp_ws/devel_isolated/setup.bash
catkin_make
```

### Troubleshooting

If you encounter any problems, please contact the maintainer
(c.dondrup@hw.ac.uk)