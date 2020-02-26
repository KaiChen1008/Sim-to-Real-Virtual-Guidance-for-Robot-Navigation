Control Policy Virtual Environment
===================================

Prerequisites
-------------

- Python 3.5 or higher
- Linux-based terminal
- ROS melodic version

Create a Virtual Environment
-----------------------------

We recommend using a virtual environment to avoid any potential conflicts with your global configuration.

First, create a virtual environment via virtualenv:

.. code-block:: bash

    virtualenv -p=path-to-python3 rlenv

Activate the virtual environment:

.. code-block:: bash

    source rlenv/bin/acitvate

Then, install all required dependencies via pip:

.. code-block:: bash

    (rlenv)$ pip install -r requirements.txt

You can exit your virtualenv after installation/using:

.. code-block:: bash

    (rlenv)$ deactivate
