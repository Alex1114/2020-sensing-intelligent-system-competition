From argnctu/sis_2020:tx2

WORKDIR /home/sis/sis_competition_2020
RUN rm -rf ./catkin_ws/src/*

COPY competition_modules/ ./catkin_ws/src
COPY task.launch ./catkin_ws/src/competition_modules/core/sis_task/launch/
COPY catkin_make.sh .
COPY environment.sh .
COPY run_task.sh .
COPY start_tx2.sh .
COPY set-ros-ip.sh .


RUN mkdir -p /home/sis/.cache/torch/checkpoints && cd /home/sis/.cache/torch/checkpoints \
&& wget https://download.pytorch.org/models/vgg16-397923af.pth

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash"