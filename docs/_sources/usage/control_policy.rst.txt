Control Policy Module
======================

Launch RL model
----------------

Run the ``control_policy.py`` script to load the trained model and process the images.

RL module would receive images from perception module and output corresponding actions to the AGV controller.


First of all, launch ROS in the terminal:

.. code-block:: bash

    roscore

Then, you need to activate the RL environment:

.. code-block:: bash

    # Activate the RL Envrionment
    source rlenv/bin/activate

After activation, use the following command to execute our code.

.. option:: Argument

.. code-block:: bash

    --port: Port that connect to the perception module.
    If not specified, runs on default arguments.


.. option:: Example Usage


.. code-block:: bash

    # Run listener.py
    (rlenv)$ python3 new_listener.py



Launch Goal-Receiver
----------------------
Goal-Receiver will receive a success message when the AGV reaches the goal. After that, the message will be sent to the AGV controller immediately to stop the AGV.

.. option:: Argument

.. code-block:: bash

    --port: Port that connect to the planner module.
    # If not specified, runs on default arguments.


.. option:: Example Usage

.. code-block:: bash

    (rlenv)$ python dummy.py

Launch AGV Controller
----------------------

Run the ``controller.py`` script to launch the AGV controller.

.. code-block:: bash

    (rlenv)$ python controller.py

