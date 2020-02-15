How to Train an RL Agent
========================

Why in Virtual Environment
--------------------------

Training environment plays an important role in Reinforcement Learning.


It’s impractical to train our agent in the real world due to the following problems:

- Training in the real world may cause great damage, and the cost is unaffordable.
- Training RL agents in the real world is time-consuming.


Although training in virtual environments would be the recommended choice, there still exists a few issues:

- Differences between camera images and computer-simulated images.
- Gap between the virtual environment and the real world.

To solve these problems, we use segmentation to classify different objects in real world.


Also, we change its color to corresponding solid color mapping to the virtual environment.


Training Environment Set-Up
---------------------------

We set up a virtual environment in Unity3D engine and use ML-Agents toolkit to connect the python code and the Unity C# environment.

Please refer to the `ML-Agents <https://github.com/Unity-Technologies/ml-agents>`_ official websites for further information.


**Start Training**

Start your training with

.. code-block:: bash

    mlagents-learn example_config.yaml --env=example.x86_64 --run-id=example --train


The following are some examples of training setting:


**Brain Parameters**


In our framework, we only use a monocular camera to handle the navigation. Hence, we set Vector Observation to 0 and add one Visual Observation in Player Brain.


**Vector Action**


We set Space Type as Discrete, Branch Size to 1, and Branch 0 Size to 3. Three actions represent move forward, turn left and turn right respectively.


**Training Configuration**

We set Width to 120, Height to 80, Quality Level to 1, Time Scale to 1, and Target Frame Rate is set to -1.


**Inference Configuration**

We set Width to 1280, Height to 720, Quality Level to 5, Time Scale to 1, and Target Frame Rate is set to 60.


Training method
----------------

**Virtual Environment**


We create 20 different maps with various obstacles and random goals. The full package can be downloaded `here <https://drive.google.com/file/d/1xYw3JfpTFHNeaovh_CIjARwDXeYZQVRq/>`_ . In this way, the agent is able to simultaneously avoid obstacles and act accordingly with our virtual guidance scheme even in unfamiliar environments.


**Reward Function**


The agent gets ‘+1’ reward if it reaches the goal, and gets ‘-1’ if it collides with the obstacles.



(Optional: the time penalty can be taken into consideration if the agent takes too much time to reach the goal.)



Computational Platform
----------------------

We train our agent on the server with an ``Intel i7-6700 CPU``, with ml-agents version ``0.9`` and tensorflow version ``1.7.1``.












