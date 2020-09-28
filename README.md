# XBOX ONE ROS JOY TELEOP

## This is node for teleop robot with ros 

I will do updates soon but as for now this is enough for me

## Install 

I have used this [repo](https://github.com/paroj/xpad) for xbox pad mapping

```
sudo git clone https://github.com/paroj/xpad.git /usr/src/xpad-0.4
sudo dkms install -m xpad -v 0.4
```

To test if joy works use `jstest /dev/input/js{0}`

## Usage

```
rosrun joy joy_node
rosrun xbox_one_joy teleop_joy
```

OR

```
roslaunch xbox_one_joy_ros joy_teleop.launch
```