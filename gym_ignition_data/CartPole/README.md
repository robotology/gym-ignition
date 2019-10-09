# CartPole model

This folder contains a parametric [CartPole.xacro](CartPole.xacro) model developed in [xacro](http://wiki.ros.org/xacro).

#### Generate the urdf

In order to generate the urdf you need to install `ros-${ROS_VERSION}-xacro` (tested with `melodic`):

```bash
xacro CartPole.urdf.xacro > CartPole.urdf
```

#### Generate the sdf

In order to generate the urdf you need to install `gazeboX` package with `X <= 9`:

```bash
gz sdf -p CartPole.urdf > CartPole.sdf
```
