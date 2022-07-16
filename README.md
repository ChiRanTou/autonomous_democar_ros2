# 自动驾驶控制算法学习demo
## 环境配置：Ubuntu 20.04 + ROS2 foxy
## 前期准备: 安装qpOASES解算器
1.下载qpOASES
```
git clone https://github.com/coin-or/qpOASES.git
```
2.进入文件夹并进行安装
```
cd qpOASES
cmake .
make
sudo make install
```
安装成功后可以在/usr/loca/include中看到
## 使用DEMO
### 编译demo
1. 进入demo文件夹
