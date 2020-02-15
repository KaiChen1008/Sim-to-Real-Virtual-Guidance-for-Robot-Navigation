Navigation Package
==================

Installation Packages
---------------------

Install turtlebot3.

.. code-block:: bash

    sudo apt install ros-melodic-turtlebot3


Install turtlebot3_navigation.

.. code-block:: bash

    sudo apt install ros-melodic-turtlebot3-navigation

Install global_planner.

.. code-block:: bash

    sudo apt install ros-melodic-global-planner

Install hector_slam.

.. code-block:: bash

    sudo apt install ros-melodic-hector-slam


Copy the provided pose_tf folder to ~/catkin_ws/src.

Build the workspace.

.. code-block:: bash

    cd ~/catkin_ws
    catkin_make


Replace the ``turtlebot3_navigation`` and ``turtlebot3_bringup`` folders under ``~/opt/ros/melodic/share`` with the ones provided.


