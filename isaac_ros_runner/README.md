# isaac_ros_runner

Github actions server runner based from [Docker Github Actions Runner](https://github.com/myoung34/docker-github-actions-runner)

# Before install

You need a [Github token PAT](https://developer.github.com/v3/actions/self_hosted_runners/#create-a-registration-token)

the following scopes are selected:

* repo (all)
* workflow
* admin:org (all) (**mandatory for organization-wide runner**)
* admin:public_key - read:public_key
* admin:repo_hook - read:repo_hook
* admin:org_hook
* notifications

# Install

Before to run the `installer.sh` script you need install and setup your NVIDIA Jetson to work with docker-compose. If you did not install **python3 pip**, please run this script on your *NVIDIA Jetson* otherwhise skip this first step.

```
sudo apt-get install -y python3-pip
```

Install docker-compose on your NVIDIA Jetson:

```
sudo apt-get install -y libffi-dev python-openssl libssl-dev
sudo -H pip3 install -U pip
sudo pip3 install -U docker-compose
```

Second step you need to add docker permission in your user:
```
sudo usermod -aG docker $USER
```
**PLEASE NOTE** You need to reboot your board!

Follow the installer running, from **`isaac_ros_runner`** folder:

```
bash installer.sh
```

# Run github action runner

from **`isaac_ros_runner`** folder:

```
docker-compose  up -d
```

# Stop Github action runner

from **`isaac_ros_runner`** folder:

```
docker-compose down
```