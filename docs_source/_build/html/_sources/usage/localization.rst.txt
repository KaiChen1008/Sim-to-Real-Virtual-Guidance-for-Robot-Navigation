Localization Module
===================

Set up Map File
---------------

Move your map file (``file.bin``) to ``home`` directory.

.. code-block:: bash

    cp path_to_your_map/file.bin ~/

Setup Environment
-----------------

.. code-block:: bash

    export ROS_PACKAGE_PATH=path_to_workspace/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2:${ROS_PACKAGE_PATH}'


Run ORB-SLAM2
--------------

Run ORB SLAM2 with the following command.

.. code-block:: bash

    rosrun ORB_SLAM2 Monopub path_to_ORBvoc path_to_setting_yaml -1 camera_topic

In our situation, the command is :


.. code-block:: bash

    ~/Desktop/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Asus.yaml -1 /zed/left/raw_image'


Run pose convertor
------------------

convert the ORB SLAM2 pose to rviz coordination.


.. code-block:: bash

    python pose_convertor.py
