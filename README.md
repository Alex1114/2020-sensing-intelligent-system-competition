# sis_competition_task  
  
[Tutorial](https://drive.google.com/drive/folders/1twHdpltCn8hlASIWL9gCv0urSpLBvABC?usp=sharing)
[Locobot NUC repo](https://github.com/Alex1114/2020-sensing-intelligent-system-competition-NUC)

## Files:

- competition_modules

    - core
        
    - place_to_box
        
    - pose_estimate_and_pick   
            
    - object_detection

    - task3(navigation)                
              
- README.md            

- Dockerfile            

- docker_run.sh       

- catkin_make.sh       

- environment.sh        

- docker_build.sh       (If you want to build docker file, please execute/source this shell)

## Run task1~task4  (Do it in TX2-docker)
Tool
```
source docker_run.sh
source docker_join.sh
source catkin_make.sh
source environment.sh
```
  
- Terminal1
```
source start_tx2.sh
```  
  
- Terminal2
```
roslaunch ojject_detection task1.launch
```
  
- Terminal3
```
roslaunch astar navigation.launch
```
  
- Terminal4
```
rosrun sis_task all.py
```  

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
