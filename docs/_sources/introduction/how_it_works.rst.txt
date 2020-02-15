How It Works
============
Our framework consists of 4 parts: 
the Perception module, Localization module, Planner module, and Control policy module.


Perception Module
-----------------
The perception module translates the agent's view (in this case, an image) into comprehensible segmented chunks.

Google Deeplab V3 on NVIDIA Jetson Xavier is recommended in this project, but any semantic segmentation model can be used.


Localization Module
--------------------

The localization module gives the current pose of the robot according to the environment features.

We use ORB-SLAM2, a high accuracy visual SLAM algorithm that only requires the input of a single monocular camera.

Planner Module
--------------
The planner module generates a path according to the current robot's pose and the goal based on the A* algorithm.

It is also responsible for communicating with the perception module regarding the placement of the "virtual guide".

Control Policy Module
----------------------
The control policy module acts as a local planner in our framework. Simply put, it is a "robot controller", which mainly focuses on avoiding obstacles and following the virtual guide, leading the robot to its goal.

In the training phase, we employ the PPO (Proximal Policy Optimization) algorithm to train the agent. Furthermore, we set up a virtual environment using Unity to simulate the segmented images (real-world).



Details
-------

For more details, please refer to our `technical report <https://www.memtest86.com>`_ .
