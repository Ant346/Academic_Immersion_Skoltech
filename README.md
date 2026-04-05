# Academic_Immersion_Skoltech

ROS2 Jazzy + GigE camera in Docker.

Compose is intentionally minimal: one service that only publishes topics.

## Prerequisites

- Linux host with Docker Engine and Docker Compose plugin.
- Camera connected to host network.
- SDK folder present next to this repository:
	- `../linuxSDK_V2.1.0.45(240521)`

## Start Topic Publisher

From this directory:

```bash
docker compose build
docker compose up -d
```

This starts a single container `gige-publisher` that:

- builds/updates package `gige`,
- runs `ros2 launch gige gige_launch.py`,
- continuously publishes camera topics.

Check topics:

```bash
docker compose logs -f gige-publisher
docker compose exec -T gige-publisher bash -lc 'source /opt/ros/jazzy/setup.bash && source /work/Academic_Immersion_Skoltech/aida-camera/install/setup.bash && ros2 topic list'
```

Expected camera topics include:
- `/GigE/Camera_1/color`
- `/GigE/Camera_1/camera_info`

## Optional: View Image Manually

If you want to test the image viewer, open GUI access and run it manually in the same container:

```bash
xhost +local:docker
docker compose exec gige-publisher bash

source /opt/ros/jazzy/setup.bash
source /work/Academic_Immersion_Skoltech/aida-camera/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Stop everything:

```bash
docker compose down
```

