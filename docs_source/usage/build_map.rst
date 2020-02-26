How to Build a Map
==================

Run Localization Module
-----------------------

Launch Localization module with following commands to start build a new map.

.. code-block:: bash

    export ROS_PACKAGE_PATH=path_to_workspace/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2:${ROS_PACKAGE_PATH}'

    rosrun ORB_SLAM2 Monopub path_to_ORBvoc path_to_setting_yaml -1 camera_topic 0 1


Open a new terminal and execute map generator.

.. code-block:: bash

    export ROS_PACKAGE_PATH=path_to_workspace/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2:${ROS_PACKAGE_PATH}'

    rosrun ORB_SLAM2 Monosub 10 0.5 20 -10 20 -10 0.55 0.5 1 5


Fine-tune
-----------

We provide a scirpt that can re-generate a 2D map from a point cloud.

.. code-block:: bash

    python pointCloudToGridMap2D.py


Run the scirpt with ``--help`` to know details.










