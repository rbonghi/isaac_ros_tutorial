# Scripts

In this folder there are all scripts to works with NVIDIA Isaac ROS GEMs.

## Fix NVIDIA Jetson to build Isaac ROS packages

To build Isaac ROS on your board, you need to add a special attribute on the docker builder, to enable the **jetson multimedia api** and set the default runtime `"default-runtime": "nvidia"`

**Please note:** This script will **restart** the docker service.

```
bash scripts/fix_jetson_docker.sh
```

## Build and push docker

If you want build one of the tutorial in this page, please run:

```
bash scripts/build_docker.sh <FOLDER_NAME>
```

You need to select which folder you want build.