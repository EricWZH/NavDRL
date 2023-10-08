# svar_summercamp

为了让大家能更快实现基础功能，这个库中实现了以下功能，并通过[Svar](https://github.com/zdzhaoyong/Svar)提供了多语言调用的接口：

* 运动仿真： src/simulate.cpp；
* 环境&传感器仿真： src/world.cpp；
* 基础计算类，包含坐标系变换等： src/gslam.cpp；
* icp类：包含icp,plicp合到了一起见src/plicp.cpp；
* 点云处理函数： 坐标变换、采样、拼接见src/plicp.cpp；
* tf坐标系插值类： tinytf.cpp;


