ORB-SLAM2
============

Install Dependency Packages
-----------------------------

.. code-block:: bash

    sudo apt install -y \
    libboost-all-dev \
    build-essential \
    cmake \
    libgtk2.0-dev \
    pkg-config \
    python-dev python-numpy \
    libavcodec-dev libavformat-dev libswscale-dev \
    libglew-dev

Install Eigen3
--------------

.. code-block:: bash

    sudo apt install libeigen3-dev
    cd /usr/include/eigen3
    sudo cp Eigen/ .. -R


Install OpenCV 3.2
------------------

Specify the opencv version.

.. code-block:: bash

    export CV3_RELEASE=3.2.0

Clone the opencv source code.

.. code-block:: bash

    git clone https://github.com/opencv/opencv.git /tmp/opencv
    git clone https://github.com/opencv/opencv_contrib.git /tmp/opencv_contrib


Install dependencies.

.. code-block:: bash

    sudo apt update
    sudo apt install -y liblapacke-dev 
    sudo apt install -y libeigen3-dev

Build and install opencv3.

.. code-block:: bash

    cd /tmp/opencv
    git checkout ${CV3_RELEASE}
    cd /tmp/opencv_contrib
    git checkout ${CV3_RELEASE}
    mkdir -p /tmp/opencv-${CV3_RELEASE}-build
    cd /tmp/opencv-${CV3_RELEASE}-build
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DWITH_CUDA=OFF \
          -DCMAKE_INSTALL_PREFIX=/usr/local/opencv-${CV3_RELEASE} \
          -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
          /tmp/opencv
    sudo make install -j`proc`

Clean up.

.. code-block:: bash

    rm -rf /tmp/opencv /tmp/opencv_contrib
    sudo rm -rf /tmp/opencv-${CV3_RELEASE}-build

Install Pangolin
------------------
Install dependencies.


.. code-block:: bash

    sudo apt install -y libglew-dev \
    cmake \
    libboost-dev \
    libboost-thread-dev \
    libboost-filesystem-dev \
    libxkbcommon-dev

Clone the source code.

.. code-block:: bash

    git clone https://github.com/stevenlovegrove/Pangolin

Build Pangolin.

.. code-block:: bash

    mkdir build && cd build
    cmake ..
    make
    sudo make install -j`proc`


Install Doxygen
---------------
.. code-block:: bash

    sudo apt-get install -y doxygen



Install ORB-SLAM2
-----------------

Please download our modification of ORB-SLAM2 which can restore maps, publish poses and construct both 2D and 3D maps simultaneously.

Move the ORB-SLAM2 into ``src`` directory of your workspace.

.. code-block:: bash

    cp -r ORB_SLAM2 `path_to_workspace_src`


Go into the ORB_SLAM2 directory and execute ``build.sh``.

.. code-block:: bash

    source `path_to_workspace`/devel/setup.bash
    cd ORB_SLAM2
    chmod +x build.sh
    ./build.sh

Build the ROS nodes.

.. code-block:: bash

    source `path_to_workspace`/devel/setup.bash
    chmod +x build_ros.sh
    ./build_ros.sh


Set up Configuration File
-------------------------

Specify parameters in a yaml file, including camera parameters, map file, etc.
Camera parameters include fx, fy, kx and ky which can be found in the camera (or lens) documentation.

We also provide our configuration file in ``ORB_SLAM2/Examples/ROS/``.