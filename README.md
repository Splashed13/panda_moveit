# panda_moveit
ROS package containing scripts utilizing MoveIt specifically MoveIt Commander interface with python, to be run in a workspace where moveit has been installed from source and panda_moveit_config has been cloned and built.

![arm4](https://user-images.githubusercontent.com/71549279/219283672-5038713b-be5f-4694-a2b1-368e434a1e52.gif)

Run by entering the following in a terminal

```
roslaunch panda_moveit run_sim.launch
```

and then the following in another terminal for the cpp code

```
rosrun panda_moveit pick_and_place
```

or for the python code 

```
rosrun panda_moveit pick_and_place.py
```

