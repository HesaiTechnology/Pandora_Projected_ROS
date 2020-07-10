# Pandora_Projection_ROS

## Build
```
mkdir -p rosworkspace/src
cd rosworkspace/src
git clone https://github.com/HesaiTechnology/Pandora_Projection_ROS.git --recursive
cd ../
catkin_make
```

## Run
```
source devel/setup.sh
roslaunch pandora_projection pandora_projection.launch
```

## ROS Topic name
```
/pandora_projection0
/pandora_projection1
/pandora_projection2
/pandora_projection3
/pandora_projection4
```
