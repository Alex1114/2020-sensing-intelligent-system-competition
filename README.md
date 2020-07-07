# sis_competition_task_template

## About this template

You need to submit by using this template for each task.

## Files:

- competition_modules

    - core
        
    - place_to_box
        
    - pose_estimate_and_pick   
            
    - object_detection

    - task3                
              
- README.md             (You don't need to modify this file)

- Dockerfile            (You don't need to modify this file)

- run_task.sh           (You don't need to modify this file)

- docker_run.sh         (You don't need to modify this file)

- catkin_make.sh        (You don't need to modify this file)

- environment.sh        (You don't need to modify this file)

- task.launch    (You have to determine which node you need to launch in the task and write in this file)

- docker_build.sh       (If you want to build docker file, please execute/source this shell)

## How to use task3 pkg
```
roslaunch astar navigation.launch
```

to go to plate number ex: plate 0
```
rosservice call /to_position "pos: 0"
```

to adjust the rotation of the locobot ex: plate 0
```
rosservice call /get_pose "plate: 0"
```

## How to build docker image:
```
tx2 $ cd [your sis_competition_template path]
tx2 $ source docker_build.sh
```
***If docker is already login with other account, please logout first.***
```
tx2 $ docker logout
```
***Type your dockerhub's account and password.***
```
tx2 $ docker login
tx2 $ docker tag sis_competition_2020:latest sis_competition_2020:[task_name]
tx2 $ docker push [dockerhub account]/sis_competition_2020:[task_name]
```
## How to run
```
tx2 $ cd [your sis_competition_template path]
tx2 $ source docker_run.sh [task name]
```
***After enter container, you need to run this command once.***
```
tx2 docker $ source catkin_make.sh 
```
***Run task***
```
tx2 docker $ source run_task.sh
```
***If you want to enter same container, run this.***
```
tx2 $ source docker_run.sh same
tx2 docker $ source environment.sh (remember!！)
```
