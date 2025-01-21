按照官方教程一步步配置
https://learn.microsoft.com/zh-cn/windows/wsl/

## WSL 连接宿主机器设备
https://learn.microsoft.com/zh-cn/windows/wsl/connect-usb#attach-a-usb-device

定制内核  
https://blog.csdn.net/breaksoftware/article/details/140752840


usbipd list  
使用命令 usbipd bind 来共享设备，从而允许它附加到 WSL。 这需要管理员权限。    
usbipd bind --busid 4-4  

linux系统加载vhci_hcd模块  
sudo modprobe vhci_hcd  

usbipd attach --wsl --busid <busid>  
usbipd detach --busid <busid>  


### ./bashrc
```sh
source /opt/ros/jazzy/setup.bash
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib:$LD_LIBRARY_PATH
export PATH=$PATH:/usr/local/cuda/bin


export PATH=$PATH:/usr/local/julia-1.7.2/bin/

export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/gz_ws/src/ardupilot_gazebo/models:$HOME/ros2_ws/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
export MAP_SERVICE="MicrosoftHyb"

export DISPLAY=localhost:0.0
# export DISPLAY=$(grep -m 1 nameserver /etc/resolv.conf | awk '{print $2}'):0
# export DISPLAY="`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0"
# export DISPLAY="`sed -n 's/nameserver //p' /etc/resolv.conf`:0"
export DISPLAY=$(ip route|awk '/^default/{print $3}'):0.0
#export DISPLAY=172.24.112.1:0

export ROS_HOSTNAME=$(ip route|awk '/^default/{print $3}')
#export ROS_MASTER_URI=http://192.168.5.142:11311


# 确保 dbus 服务在启动时自动运行
# if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
#     eval "$(dbus-launch --sh-syntax)"
#     echo "DBUS_SESSION_BUS_ADDRESS=$DBUS_SESSION_BUS_ADDRESS" >> ~/.bashrc
#     echo "DBUS_SESSION_BUS_PID=$DBUS_SESSION_BUS_PID" >> ~/.bashrc
# fi

# if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
eval "$(dbus-launch --sh-syntax)"
export DBUS_SESSION_BUS_ADDRESS=$DBUS_SESSION_BUS_ADDRESS
export DBUS_SESSION_BUS_PID=$DBUS_SESSION_BUS_PID
# fi

#确保共享库路径包含Pangolin库的安装路径
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

export LD_PRELOAD=/usr/local/lib/liboctomap.so:$LD_PRELOAD

export ROS_HOSTNAME=172.24.114.52
export ROS_MASTER_URI=http://172.24.114.52:11311
```

### WSL配置
当前配置（其他位于WSL Settings）
```
[wsl2]
kernel=D:\\File_wsl\\vmlinux\\vmlinux
guiApplications=true
```

.wslconfig
```.wslconfig
[wsl2]
memory=4GB
swap=4GB


[wsl2]
kernel=D:\\File_wsl\\vmlinux\\vmlinux

使用镜像模式
[experimental]
autoMemoryReclaim=gradual  
networkingMode=mirrored
dnsTunneling=true
firewall=true
autoProxy=true
在默认情况下，wsl的网络模式是net模式，在这种模式下你在cmd中输入ipconfig会看到以太网适配器 vEthernet (WSL (Hyper-V firewall))，于是直接在~/.bashrc中应用export DISPLAY=xxx(那里面的ip地址)就可以，不会出现任何问题
但是！一旦你使用了镜像网络，这个适配器就失效了，但是在你输入ipconfig的时候，这个适配器依然会被显示出来，除非重启电脑它才会消失。
而取而代之的是无线局域网适配器 WLAN，也就是说你要将export DISPLAY=后面的IP地址改成它的IPV4地址。使用export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0从 /etc/resolv.conf 文件中获取的是默认网关地址（其充当局域网（LAN）和外部网络（如互联网）之间的中介）而不是ipv4地址
```

---
## WSL使用图形化界面
1. win安装xserver
   
- 安装vcxsrv

config.xlaunch(自动生成配置文件)  
```xml
<?xml version="1.0" encoding="UTF-8"?>
<XLaunch WindowMode="MultiWindow" ClientMode="NoClient" LocalClient="False" Display="-1" LocalProgram="xcalc" RemoteProgram="xterm" RemotePassword="" PrivateKey="" RemoteHost="" RemoteUser="" XDMCPHost="" XDMCPBroadcast="False" XDMCPIndirect="False" Clipboard="True" ClipboardPrimary="True" ExtraParams="" Wgl="True" DisableAC="True" XDMCPTerminate="False"/>
```

startXubuntu.bat

```startXubuntu.bat
set ws = WScript.CreateObject("WScript.Shell")
ws.Run """D:\Program Files\VcXsrv\scrips\startXubuntu.bat""", 0```

xubuntu.vbs

```xubuntu.vbs
set ws = WScript.CreateObject("WScript.Shell")
ws.Run """D:\Program Files\VcXsrv\scrips\startXubuntu.bat""", 0
```

连接到  
以太网适配器 vEthernet (WSL (Hyper-V firewall)):
   IPv4 地址 . . . . . . . . . . . . : 172.24.112.1

禁用镜像模式
```sh
./bashrc
# export DISPLAY=$(grep -m 1 nameserver /etc/resolv.conf | awk '{print $2}'):0
# export DISPLAY="`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0"
# export DISPLAY="`sed -n 's/nameserver //p' /etc/resolv.conf`:0"
export DISPLAY=$(ip route|awk '/^default/{print $3}'):0.0
```

sudo startxfce4

---

2. 解决gnome-terminal无法启动问题（mavproxy[Errno 111]Connection refused sleeping）

先启动自己安装的xsrv

dbus-launch gnome-terminal

```sh
./bashrc
# 确保 dbus 服务在启动时自动运行
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    eval "$(dbus-launch --sh-syntax)"
    echo "DBUS_SESSION_BUS_ADDRESS=$DBUS_SESSION_BUS_ADDRESS" >> ~/.bashrc
    echo "DBUS_SESSION_BUS_PID=$DBUS_SESSION_BUS_PID" >> ~/.bashrc
fiDBUS_SESSION_BUS_ADDRESS=unix:path=/tmp/dbus-MhRCKi6OME,guid=8c4fe1d46f582f0dd685d22266eaf83c
DBUS_SESSION_BUS_PID=721
```
```
mavproxy.py --map --console
MAV> set moddebug 3
MAV> module load map
```
## WSL设置端口转发

win终端

wsl -- ifconfig eth0

 inet 172.24.114.52  netmask 255.255.240.0  broadcast 172.24.127.255
 注意inet后边的就是我们虚拟机的ip地址了。

netsh interface portproxy add v4tov4 listenport=26790 listenaddress=127.0.0.1 connectport=26790 connectaddress=172.24.114.52

删除映射端口命令
 netsh interface portproxy delete v4tov4 listenport=你的端口 listenaddress=127.0.0.1 

ros2:
 https://blog.csdn.net/qq_43365306/article/details/138618983