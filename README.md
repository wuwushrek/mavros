MAVROS with modifications for testing our learning-based control algorithms
========================================================================

This provides a description on how to setup mavlink, mavros, and mavlink-router for using our algorithms.
MAVROS is a ROS package that provides a bridge between ROS and MAVLink. MAVLink is a lightweight protocol for communicating with micro air vehicles (UAVs) and other robotic systems. MAVLink-router is a tool that allows to route MAVLink messages between different interfaces.

## MAVROS and MAVLink Pre-requisites

We first setup the dependencies for mavros and mavlink. Then, we modify both packages and build them.
This part assumes that you have ROS, and a catkin workspace already set up to work with catkin tools.


1. If not, to install ROS, follow the instructions at http://wiki.ros.org/noetic/Installation/Ubuntu. Then, install catkin tools as below:
```bash
sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y
```
Set up catkin workspace, if needed
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src
```

2. Setup mavlink and mavros dependencies
```bash
rosinstall_generator --rosdistro noetic mavlink | tee /tmp/mavros.rosinstall # Use the corresponding ROS distro, e.g. noetic on my Ubuntu 20.04
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall # Get the latest source
wstool merge -t src /tmp/mavros.rosinstall # Make sure you are in the catkin ws directory
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
```

3. Install geographiclib datasets
```bash
cd ~/
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh # Or sudo ./install_geographiclib_datasets.sh
```

## MAVLINK Modifications

At this point, mavlink should be in catkin_ws/src/mavlink. We will modify the mavlink message definitions to include our custom messages.

1. If you are using a different version of mavlink, you will need to modify the message definitions in the corresponding version of mavlink.
You can obtain another mavlink as follows:
```bash
cd ~/catkin_ws/src
git clone https://github.com/mavlink/mavlink-gbp-release.git
mv mavlink-gbp-release mavlink
cd mavlink
git checkout release/noetic/mavlink # Or whatever branch you want
```

2. Then, you can modify the message definitions in the message_definitions/v1.0/common.xml file.
You can use any text editor to modify the file. For example, you can use the following command to open the file in VS Code:
```bash
cd ~/catkin_ws/src/mavlink
code message_definitions/v1.0/common.xml
```

3. Now, we will modify the .xml file by inserting the snipplet below
Search for SMART_BATTERY_INFO and copy and paste the tag below before the SMART_BATTERY_INFO message and save the file.
```xml
    <message id="367" name="MPC_FULL_STATE">
      <description>Full MPC State used for offline mpc-based control</description>
      <field type="uint64_t" name="time_usec" units="us">Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.</field>
      <field type="float" name="x" units="m">X Position in POSE_FRAME_NED</field>
      <field type="float" name="y" units="m">Y Position in POSE_FRAME_NED</field>
      <field type="float" name="z" units="m">Z Position in POSE_FRAME_NED</field>
      <field type="float" name="vx" units="m/s">X Speed in POSE_FRAME_NED</field>
      <field type="float" name="vy" units="m/s">Y Speed in POSE_FRAME_NED</field>
      <field type="float" name="vz" units="m/s">Z Speed in POSE_FRAME_NED</field>
      <field type="float" name="qw">Qw quaternion : Body frame NED to pose_frame_ned</field>
      <field type="float" name="qx">Qx quaternion : Body frame NED to pose_frame_ned</field>
      <field type="float" name="qy">Qy quaternion : Body frame NED to pose_frame_ned</field>
      <field type="float" name="qz">Qz quaternion : Body frame NED to pose_frame_ned</field>
      <field type="float" name="wx" units="rad/s/s">Rollspeed : Body frame NED</field>
      <field type="float" name="wy" units="rad/s/s">Pitchspeed : Body frame NED</field>
      <field type="float" name="wz" units="rad/s/s">Yawspeed : Body frame NED</field>
      <field type="float" name="m1">Motor 1 input</field>
      <field type="float" name="m2">Motor 2 input</field>
      <field type="float" name="m3">Motor 3 input</field>
      <field type="float" name="m4">Motor 4 input</field>
      <field type="float" name="m5">Motor 5 input</field>
      <field type="float" name="m6">Motor 6 input</field>
    </message>
    <message id="368" name="MPC_MOTORS_CMD">
      <description>Full MPC Normalized motors commands</description>
      <field type="uint64_t" name="time_usec" units="us">Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.</field>
      <field type="float[6]" name="motor_val_des">Motor imputs between 0 and 1</field>
      <field type="float[4]" name="thrust_and_angrate_des">Thrust and angular rate desired command. T, wx, wy,wz. T is between 0 and 1 while wx, wy, wz have standard units</field>
      <field type="uint8_t" name="mpc_on">Specify if the state of the mpc</field>
      <field type="uint8_t" name="weight_motors">Constant parameter weighting if we track the angrate_des or motor output directly.Value between 0 and 100 </field>
    </message>
```

4. Now, we build mavlink
```bash
cd ~/catkin_ws/
catkin build mavlink
```

5. We install pymavlink
```bash
sudo apt-get install libxml2-dev libxslt-dev
sudo python3 -m pip install --upgrade future lxml
cd ~/catkin_ws/src/mavlink/pymavlink
python3 -m pip install .
```

## MAVROS Modifications

Clone the mavros repository and checkout the mpc_franck branch
```bash
cd ~/catkin_ws/src
git clone https://github.com/wuwushrek/mavros.git
cd mavros
git checkout mpc_franck
cd ~/catkin_ws/
catkin build mavros mavros_extras
```

## MAVLINK-ROUTER

Clone the mavlink-router repository and install dependencies
```bash
cd ~/Documents
git clone https://github.com/mavlink-router/mavlink-router.git
cd mavlink-router
git submodule update --init --recursive
sudo apt install git ninja-build pkg-config gcc g++ systemd
python3 -m pip install meson
```

1. We need to update the common.xml file to include the new messages we created. We are going to copy the common.xml file from the mavlink folder we created earlier
```bash
cp ~/catkin_ws/src/mavlink/message_definitions/v1.0/common.xml modules/mavlink_c_library_v2/message_definitions
# Make sure ~/.local/bin is in your PATH
meson setup build .
ninja -C build
# Assuming ~/.local/bin is in your PATH, add the binary to your path so you can run it from anywhere
cp build/mavlink-routerd ~/.local/bin
```