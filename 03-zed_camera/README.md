# Isaac ROS - ZED camera

# Build

```
bash scripts/build_docker.sh 03-zed_camera
```

# Run docker

```
docker run --runtime nvidia --network host --privileged rbonghi/isaac-ros-tutorial:zed-camera
```

Option explained:
* --runtime nvidia will use the NVIDIA container runtime while running the l4t-base container
* --network host To read all topics outside the container need to share the same network to the host
* -v is the mounting directory
*  --privileged Allow access to host