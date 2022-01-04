# Isaac ROS - Argus camera

Example to setup a Dockerfile with all Isaac ROS packages to run an Isaac ROS argus camera

# Build

```
bash script/build_docker.sh 01-argus_camera
```

# Run docker

```
docker run -v /tmp/argus_socket:/tmp/argus_socket --runtime nvidia --network host rbonghi/isaac-ros-tutorial:argus-camera
```

Option explained:
* --runtime nvidia will use the NVIDIA container runtime while running the l4t-base container
* --network host To read all topics outside the container need to share the same network to the host
* -v is the mounting directory