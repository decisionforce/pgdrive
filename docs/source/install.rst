.. _install:

######################
Installing PGDrive
######################

By leveraging the power of panda3d, PGDrive can be run on personal laptop, cluster, headless server with different OS.

Install PGDrive on MacOs, Windows and Linux
###############################################

The installation procedure on these different platforms is same and easy, we recommend to use the command following to install::

    pip install git+https://github.com/decisionforce/pgdrive.git

or you can install via::

    git clone https://github.com/decisionforce/pgdrive.git

    cd pgdrive

    pip install -e .

Install PGDrive on headless machine or cluster
#################################################
If lidar information is enough for your agent, you can also install PGDrive on your headless machine by the way we mentioned above.
However, if you want to use image to train your agent on headless machine, you have to compile from the source code of panda3d.
Follow the instructions on the main page of `panda3d <https://github.com/panda3d/panda3d>`_, and then use the command following to compile panda3d::

    python ./makepanda/makepanda.py --everything --no-x11 --no-opencv --no-fmodex --python-incdir /path/to/your/conda_env/include/ --python-libdir /path/to/your/conda_env/lib/ --thread 8 --wheel


It will give you a panda3d which can run in EGL environment without the X11 support.
Install the wheel file by::

    pip install panda3d-1.10.xxx.whl

and PGDrive will utilize the power of cluster to train your agent!

.. warning:: Compiling panda3d from source requires the **Admin permission** to install some libraries.
We are working to provide a static built panda3d for cluster users of PGDrive to make it easy to use on headless machines.

.. note:: The boolean in "pg_world_config" named "headless_image" must be set to True, when training the agent using image input.

Verify Installation
#########################
Run commands below to verify the installation::

    python -m pgdrive.tests.install_test.test_install

Successfully running this script means the PGDrive works well, and an image will be shown to help you check if PGDrive
can launch and capture image in offscreen mode

To verify the installation on cluster, run following command instead::

    python -m pgdrive.tests.install_test.test_headless_install

Please, fetch the images on cluster and check the images generated on the headless machine to ensure PGDrive can draw scene
and capture images without X11.