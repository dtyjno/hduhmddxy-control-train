## 来源：[复制网上博客]()

### ubuntu系统 opencv c++版 支持cuda 安装流程

### 计划
1. 介绍从源代码安装支持CUDA的OpenCV C++的步骤。
2. 提供安装所需的依赖项和工具的说明。
3. 详细说明从源代码编译OpenCV的步骤。
4. 提供验证安装的简单代码示例。

## 前提条件
安装系统为 ubuntu24.04  
在开始之前，请确保你已经安装了以下工具和依赖项：
- CMake
- Git
- CUDA Toolkit（确保CUDA与您的GPU兼容）
- Python（用于配置和测试）

## 步骤1：下载OpenCV和OpenCV Contrib源代码
首先，克隆OpenCV和OpenCV Contrib的源代码仓库：
```sh
git clone https://github.com/opencv/opencv.git -b 4.x
cd opencv
git clone https://github.com/opencv/opencv_contrib.git -b 4.x

git checkout 4.x
cd opencv_contrib
git checkout 4.x
cd ..
```

## 步骤2：创建构建目录
在OpenCV源代码目录中创建一个新的构建目录：
```sh
mkdir build
cd build
```


## 步骤3：配置CMake
使用CMake配置OpenCV构建选项，确保启用CUDA支持并指定OpenCV Contrib模块路径：
```sh
cmake -D CMAKE_BUILD_TYPE=Release       -D CMAKE_INSTALL_PREFIX=/usr/local       -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules       -D WITH_CUDA=ON       -D CUDA_ARCH_BIN=8.9       -D CUDA_ARCH_PTX=       -D WITH_CUDNN=ON       -D OPENCV_DNN_CUDA=ON       -D ENABLE_FAST_MATH=1       -D CUDA_FAST_MATH=1       -D WITH_CUBLAS=1       ../ #../opencv

```

```bash
cmake   -D CMAKE_BUILD_TYPE=Release\
        -D CMAKE_INSTALL_PREFIX=/usr/local\
        -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules\
        -D WITH_CUDA=ON\
        -D CUDA_ARCH_BIN=8.9\
        -D CUDA_ARCH_PTX=\
        -D WITH_CUDNN=ON\
        -D OPENCV_DNN_CUDA=ON\
        -D BUILD_opencv_dnn=ON\
        -D ENABLE_FAST_MATH=1\
        -D CUDA_FAST_MATH=1\
        -D WITH_CUBLAS=1\
        -D OpenBLAS_INCLUDE_DIR=/usr/include/x86_64-linux-gnu/\
        -D OpenBLAS_LIB=/usr/lib/x86_64-linux-gnu/libopenblas.so\
        -D Atlas_CLAPACK_INCLUDE_DIR=/usr/include/x86_64-linux-gnu/atlas\
        -D Julia_INCLUDE_DIR=~/.julia/juliaup/julia-1.10.5+0.x64.linux.gnu/include/julia\
        -D GLOG_INCLUDE_DIR=/usr/include/glog\
        -D OGRE_INCLUDE_DIR=/usr/include/OGRE\
        .

```
注意：
* `CUDA_ARCH_BIN`(rtx4060：8.9)和`CUDA_ARCH_PTX`的值应根据你的GPU架构进行调整。
* -D OPENCV_EXTRA_MODULES_PATH= /home/opencv_contrib/modules需要填写你自己的opencv_contrib路径

发现报错need enabled cudev，需要将opencv_contrib中的modules中的cudev文件夹复制到opencv/modules里
cp -r /home/opencv_contrib/modules/cudev /home/opencv/modules/

## 步骤4：编译和安装OpenCV
使用以下命令编译和安装OpenCV：
```sh
# make -j$(nproc)
make -j4
sudo make install
```



## 步骤5：配置环境变量

https://blog.csdn.net/henghuizan2771/article/details/136168673

3.配置环境  
在/etc/ld.so.conf.d/文件夹下有一个opencv.conf，里面需要写入/usr/local/lib。

```sh
cd /etc/ld.so.conf.d/
sudo touch opencv.conf
sudo sh -c 'echo "/usr/local/lib" > opencv.conf'
```
更新pkg-config：
```sh
sudo ldconfig
```
打开/etc/bash.bashrc文件，在最后两行加上这两句话

sudo gedit /etc/bash.bashrc

```sh
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
export PKG_CONFIG_PATH
```


执行这三句
```sh
source /etc/bash.bashrc
sudo apt install mlocate
sudo updatedb
```

查看opencv版本 指令：pkg-config opencv --modversion

在这里插入图片描述
解决方法：

cd /usr/local/lib
sudo mkdir pkgconfig
cd pkgconfig
sudo gedit opencv.pc
```sh
prefix=/usr/local
exec_prefix=${prefix}
includedir=${prefix}/include
libdir=${exec_prefix}/lib

Name: opencv
Description: The opencv library
Version:4.10.0
Cflags: -I${includedir}/opencv4
Libs: -L${libdir} -lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann  -lopencv_core
~                                               
```
确保将OpenCV的安装路径添加到系统的环境变量中。你可以通过以下步骤来配置环境变量：
1. 右键点击“此电脑”图标，选择“属性”。
2. 点击“高级系统设置”。
3. 在“系统属性”窗口中，点击“环境变量”。
4. 在“系统变量”部分，找到并选择“Path”，然后点击“编辑”。
5. 添加OpenCV的安装路径，例如：`C:\opencv\install\x64\vc15\bin`。
``
## 验证安装

4.测试opencv
cd ~/opencv/samples/cpp/example_cmake目录下，依次执行以下命令：
```sh
cmake .
make
./opencv_example
```

创建一个简单的C++程序来验证OpenCV是否安装成功并支持CUDA：
```cpp
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <iostream>

int main() {
    if (cv::cuda::getCudaEnabledDeviceCount() == 0) {
        std::cout << "No CUDA devices found" << std::endl;
        return -1;
    }
    cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());
    return 0;
}
```
编译并运行该程序，如果输出CUDA设备信息，说明安装成功。

恭喜你！你已经成功从源代码安装了支持CUDA的OpenCV C++。
```