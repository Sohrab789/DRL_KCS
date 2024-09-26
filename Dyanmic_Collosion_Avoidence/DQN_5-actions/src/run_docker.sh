docker run -it --rm --gpus all --name dqn_asv_dc \
    --net=host \
    --env="DISPLAY" \
    --workdir="/home/docker/DQN-ASV-dyanmic-collision/src" \
    --volume="$(pwd):/home/docker/DQN-ASV-dyanmic-collision/src" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    abhilashiit/dqn_asv_dc:2.0 bash
