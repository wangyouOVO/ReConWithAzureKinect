# ReConWithAzureKinect 
## This Project is used to reconstruct the 3D model of face by AzureKinects
<br />

<h3 align="center">How to launch this project?</h3>

<h4>1. Experimental platform</h4>
  1. Operating system: Ubuntu 20.04
 <br />
 2. Depth sensor: Azure kinect
 <br />
 3. python version: 3.8
<br />
 4. OpenCV version: 4.6.0
 <br />
 5. Open3D version: 0.16
<h4>2. Install the sensor driver on Ubuntu 20.04</h4>
<br />

```sh
wget "https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb"
wget "https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb"
wget "https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb"
sudo dpkg -i libk4a1.4_1.4.1_amd64.deb
sudo dpkg -i libk4a1.4-dev_1.4.1_amd64.deb
sudo apt install libsoundio1
sudo dpkg -i k4a-tools_1.4.1_amd64.deb
#Check whether the sensor is installed.
sudo k4aviewer
#Create the following files, so that the sensor can obtain higher-level permissions.
sudo touch /etc/udev/rules.d/99-k4a.rules
```
add txt into 99-k4a.rules

```sh

# Bus 002 Device 116: ID 045e:097a Microsoft Corp.  - Generic Superspeed USB Hub
# Bus 001 Device 015: ID 045e:097b Microsoft Corp.  - Generic USB Hub
# Bus 002 Device 118: ID 045e:097c Microsoft Corp.  - Azure Kinect Depth Camera
# Bus 002 Device 117: ID 045e:097d Microsoft Corp.  - Azure Kinect 4K Camera
# Bus 001 Device 016: ID 045e:097e Microsoft Corp.  - Azure Kinect Microphone Array

BUS!="usb", ACTION!="add", SUBSYSTEM!=="usb_device", GOTO="k4a_logic_rules_end"

ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097a", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097b", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097c", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097d", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097e", MODE="0666", GROUP="plugdev"

LABEL="k4a_logic_rules_end"

```

<h4>3. Configure the project environment with conda</h4>


1. Clone the project

```sh
git clone https://github.com/wangyouOVO/ReConWithAzureKinect.git
```

2. create python env
```sh
conda create -n 3dRec python=3.8
conda activate 3dRec
pip install open3d
pip install opencv-python
```
3. use this project
```sh
....
```
