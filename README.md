# Sim-to-Real Virtual Guidance for Robot Navigation

[![documentation_link](https://img.shields.io/badge/docs-online-brightgreen.svg)](https://hackmd.io/@dppa1008/r1L8e6AZI/)

An effective, easy-to-implement, and low-cost modular framework for robot navigation tasks.

## Features

- Automatically navigate the robot to a specific goal without any high-cost sensors.
- Based on a single camera and use deep learning methods.
- Use Sim-to-Real technology to eliminate the gap between virtual environment and real world.
- Introduce Virtual guidance to entice the agent to move toward a specific direction.
- Use Reinforcement learning to avoid obstacles while driving through crowds of people.

## Prerequisites

- Ubuntu 18.04
- gcc5 or higher
- Python 2.7.17 or higher
- Python 3.5 or higher

**Note: Both versions of Python required.**

## How It Works

<!-- ![](https://i.imgur.com/fd0u5ws.png) -->

1. Our full architecture is split into four parts: the Perception module, Localization module, Planner module and Control policy module.
2.  The perception module translates the image into comprehensible segmented chunks
3. The Localization module calculates the agent’s position.
4. The Planner module generates a path leading to the goal. This path is then communicated to the control policy module via a “virtual guide”. 
5. The Control policy module then deploys deep reinforcement learning to control the agent. 
6. For more details please refer to the [website](https://www.hackster.io/do-you-wanna-build-a-snowman/sim-to-real-virtual-guidance-for-robot-navigation-71e54a).

## Documentation

See [here](https://hackmd.io/@dppa1008/virtual_guidance_for_robot_navigation/).

## Installation

Please refer to [Manual](https://hackmd.io/@dppa1008/virtual_guidance_for_robot_navigation/https%3A%2F%2Fhackmd.io%2F%40dppa1008%2FSkxAOxyf8)

## Usage
Please refer to [Manual](https://hackmd.io/@dppa1008/virtual_guidance_for_robot_navigation/https%3A%2F%2Fhackmd.io%2F%40dppa1008%2FSkxAOxyf8)
