# Installation 
## ROS
You can find these installation instructions [here](wiki.ros.org/melodic/Installation/Ubuntu).
#### Setup your sources.list
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#### Set up your keys
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
#### Update packages and install ROS
	sudo apt update
	sudo apt install ros-melodic-desktop-full
#### Setup the environment
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc	
#### Dependencies
	sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
#### Rosdep
	sudo apt install python-rosdep
	sudo rosdep init
	rosdep update

## Ardupilot
### Installing Ardupilot and MAVProxy
#### Clone ArduPilot

In home directory:
```
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout Copter-3.6
git submodule update --init --recursive
```

#### Install dependencies:
```
sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect
```

#### Use pip (Python package installer) to install mavproxy:
```
sudo pip install future pymavlink MAVProxy
```

Open `~/.bashrc` for editing:
```
gedit ~/.bashrc
```

Add these lines to end of `~/.bashrc` (the file open in the text editor):
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```

Save and close the text editor.

Reload `~/.bashrc`:
```
. ~/.bashrc
```

Run SITL (Software In The Loop) once to set params:
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```
## Gazebo and Plugins
#### Gazebo

Setup your computer to accept software from http://packages.osrfoundation.org:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Reload software list:
```
sudo apt update
```
Install Gazebo:
```
sudo apt install gazebo9 libgazebo9-dev
```
### Install Gazebo plugin for APM (ArduPilot Master) :
```
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
git checkout dev
```
build and install plugin
```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
```
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```
Set paths for models:
```
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

#### Run Simulator
In one Terminal (Terminal 1), run Gazebo:
```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

In another Terminal (Terminal 2), run SITL:
```
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```



## Extra installation instructions start here


### Fix gazebo error 

```bash
sudo apt upgrade libignition-math2
```

### Set up Python3 and catkin workspace

```bash
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy
sudo apt install python3-pip 
sudo apt install ros-melodic-ros-numpy
```

#### Install Python3 packages

```bash
pip3 install --upgrade pip
pip3 install numpy
pip3 install future
pip3 install opencv-python
pip3 install matplotlib
pip3 install torch torchvision torchaudio
pip3 install scipy
pip3 install sklearn
```

#### Setup catkin workspace 

 ```bash
 mkdir -p ~/catkin_ws/src
 cd ~/catkin_ws
 catkin init
 wstool init src
 sudo apt install python-catkin-tools python-rosinstall-generator 
 # Installing mavros and mavlink
 rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
 rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
 wstool merge -t src /tmp/mavros.rosinstall
 wstool update -t src -j4
 rosdep install --from-paths src --ignore-src -y
 sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
 sudo apt install ros-melodic-geographic-msgs
 # Install geometry and geometry2 ros packages
 wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
 cd src
 git clone -b melodic-devel https://github.com/ros/geometry.git
 # Build catkin workspace with pyhton3 environment
 catkin build --cmake-args \
             -DCMAKE_BUILD_TYPE=Release \
             -DPYTHON_EXECUTABLE=/usr/bin/python3 \
             -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
             -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
 source devel/setup.bash 
 # To avoid typing this command always, export the above line in the .bashrc 
 echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc
 ```

- The above instructions should make sure that the workspace is built with python3 environment and all the libraries should import successfully. To make sure this, verify 

```bash
# Type python3 in a terminal and inside python3 instance verify the import statements
python3
>>> import rospy
>>> import rospkg
>>> import tf
>>> import tf2_ros
>>> import tf2_py
```

#### Clone the solution package in the src folder and build workspace with python3

```bash
cd ~/catkin_ws/src
git clone <url-to-github-repo>
# rename the cloned folder to "solution"
cd ..
catkin build --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
source devel/setup.bash
# Make the scripts executable 
cd src/package_name/scripts
sudo chmod +x *.py
```

>NOTE: Make sure to build catkin workspace using the above command with --cmake-args  and NOT just catkin build. Also DO NOT use catkin_make 



## Execution

- The entire problem is solved in 2 major parts as instructed 

  - Summer Mapping : The drone goes and explores the environment and follows the road maintaining the height limit of 20m from ground. After the successful execution of this section, the co-ordinates of the road should be available in a file which will be read by the next section of execution.
  - Winter following : The path of the road is discovered by the drone and now the car and the drone follows the road. The car position is estimated using the camera of drone.

- To execute summer mapping section 

  Launch ardupilot SITL 

  ```bash
  sim_vehicle.py -v ArduCopter -f gazebo-iris --console
  ```

  Launch the gazebo world and make sure the drone and the car are spawnned

  For better visualisation launch QGroundControl 

  Run the following (for tftree)

  ```
  rosrun solution broadcaster.py
  ```

  ```
  rosrun solution summer_mapping.py 
  ```

  This should create the path of road in a numpy array and save it

- To execute winter following section

  ```bash
  rosrun solution winter_tracking.py 
  ```

  This should complete the entire solution of problem statement

​	







