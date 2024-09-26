docker run -it --rm --gpus all --name dqn_asv_sc \
    --net=host \
    --env="DISPLAY" \
    --workdir="/home/docker/DQN-ASV-static-collision/src" \
    --volume="$(pwd):/home/docker/DQN-ASV-static-collision/src" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    abhilashiit/dqn_asv_sc:2.0 bash
