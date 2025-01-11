# This is just a template 

## Install necessary dependencies
```
sudo apt install ros-humble-rosbridge-server
sudo apt install ros-humble-rosbridge-suite
sudo apt install ros-humble-web-video-server
sudo apt install libyaml-cpp-dev 
sudo apt install ros-humble-joy

sudo apt install ros-humble-realsense2_camera 
sudo apt purge ros-humble-realsense2_camera
sudo apt purge ros-humble-realsense2-camera-msgs

cd Livox-SDK2_local && mkdir build && cmake .. && make && sudo make install
cd SOEM_local && mkdir build && cmake .. && make && sudo make install

./make.sh
```

## Notes 
Command penting:
```
sudo su (Run program menggunakan root)
ps -eo pid,comm,pri,ni,cls (Untuk melihat priority, niceness, Scheduling)

Pastikan ssh (di server) forward x11 
sudo nano /etc/ssh/sshd_config
PermitRootLogin yes
X11Forwarding yes
X11UseLocalhost yes 

Pastikan untuk auth setelah melakukan sudo su (lihar di auth_x11_ssh.txt)
```