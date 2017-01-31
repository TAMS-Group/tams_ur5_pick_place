tams_ur5_pick_place
======

This reposetory provides a demo for pick and place using tams' lab setup with the ur5.

PLACE IS NOT YET SUPPORTED!

---

__Usage__

The following two launch files will start the environment to use pick and place (place is not integrated yet).

```roslaunch tams_ur5_pick_place hardware_demo.launch```

```roslaunch tams_ur5_pick_place moveit_demo_mode.launch```

Both launch files will bring up the tams lab corner from the repo [tams_ur5_setup](https://github.com/TAMS-Group/tams_ur5_setup),
the hardware launch will also launch the object detection on the table from this repo [project15_objectrecognition](https://github.com/TAMS-Group/project15_objectrecognition).
These files also start the plan_grasps_service used by moveits pick.

---

__Source files__

plan_grasps_service.cpp:  
This is the implementation of the grasp planning service used by moveits pick. It provides a grasp from above the object.

pick_pace_demo.cpp:  
This demo is a loop where the robot uses moveits pick to grasp an object on the table lift it up and
place it on the table again, then moveing back to the home position. During this movement, the user can change
the position of the object on the table.
In the moment the place part is a stub.

pick_and_place_test.cpp:  
This file is just for testing and should not be used as a demo.
