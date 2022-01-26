# Isaac ROS - OAK-D camera

# Build

Remind, before to build add an extra cache

```
bash scripts/build_docker.sh 04-oakd_camera
```

# Run docker

```
docker run --runtime nvidia --network host --privileged rbonghi/isaac-ros-tutorial:oakd-camera
```

Option explained:
* --runtime nvidia will use the NVIDIA container runtime while running the l4t-base container
* --network host To read all topics outside the container need to share the same network to the host
* -v is the mounting directory
*  --privileged Allow access to host

## Hack docker from this folder

```
docker run --network host --privileged -v $HOME/isaac_ros_tutorial/04-oakd_camera/isaac_ros_oakd:/opt/ros_ws/src/isaac_ros_oakd -it --rm rbonghi/isaac-ros-tutorial:oakd-camera bash
```