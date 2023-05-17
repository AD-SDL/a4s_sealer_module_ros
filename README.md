# A4S Sealer Module


## Description
A repository for A4S Sealer, including user manuals and remote control interfaces.

This package guides a user to install a4s_sealer_module package, remotely control and receive feedback from the Sealer.

## User Guide
1. ### Create a workspace
- `cd ~`
<!-- - `source /opt/ros/foxy/setup.bash` -->
- Create a WEI workspace if it doesn't exist already
- `mkdir -p ~/wei_ws/src`

2. ### Git Clone brooks_xpeel_module repository
- `cd ~/wei_ws/src`
- `git clone https://github.com/AD-SDL/a4s_sealer_module.git`

3. ### Install Brooks Sealer Driver
    #### Creating a conda/venv environment.
    - `conda create -n selaer-driver python=3.9`
    - `conda activate selaer-driver`

    #### Python Install 
    - `cd ~/wei_ws/src/a4s_sealer_module/a4s_sealer_driver`
    - `pip install -r requirements.txt`
    - `pip install -e .`
    - `pip install pyserial`
    - `conda install pyserial`

    This installs a4s_sealer_driver as a package
4. ### Install Brooks Peeler Client
- `cd ~/wei_ws`
- `colcon build`
- `source install/setup.bash`

## ROS2 Launch

### Sealer Client
 - `ros2 launch a4s_sealer_client a4s_sealer_client.launch.py`
 - Port name can be specified as `ros2 launch a4s_sealer_client a4s_sealer_client.launch.py sealer_port:=/dev/ttyUSB1`
<p>&nbsp;</p>


		
