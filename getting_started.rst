.. _getting_started:

Getting Started
===============

This guide will help you get started with ROS4HRI.

Installation
------------

ROS4HRI is available as a set of ROS packages.

Packages available in ROS Humble
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following core packages can be installed directly via ``apt``:

.. code-block:: bash

    sudo apt install ros-humble-hri-msgs \
                     ros-humble-hri-actions-msgs \
                     ros-humble-libhri \
                     ros-humble-hri-rviz \
                     ros-humble-human-description

Installing other packages from source
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Other packages (like detectors) currently need to be installed from source. For example, to install the face detector:

.. code-block:: bash

    # Create a workspace
    mkdir -p ~/ros4hri_ws/src
    cd ~/ros4hri_ws/src

    # Clone the repository
    git clone https://github.com/ros4hri/hri_face_detect.git

    # Install dependencies
    cd ~/ros4hri_ws
    rosdep install --from-paths src --ignore-src -r -y

    # Build
    colcon build --symlink-install
    source install/setup.bash

.. note::
    
    Support for ROS Jazzy and Rolling is planned but not yet available.

First Steps: Command Line
-------------------------

Once you have installed the packages, you can start detecting faces.

1.  **Start the face detector:**

    You need a running camera driver (publishing to ``/image_raw`` by default).

    .. code-block:: bash

        # Terminal 1: Start your camera (example with usb_cam)
        ros2 run usb_cam usb_cam_node_exe

        # Terminal 2: Start the face detector
        ros2 run hri_face_detect hri_face_detect

2.  **Inspect the results:**

    Check the list of tracked faces:

    .. code-block:: bash

        ros2 topic echo /humans/faces/tracked

    This topic publishes a ``hri_msgs/IdsList`` message containing the IDs of currently detected faces.

    To see the Region of Interest (ROI) for a specific face (replace ``<face_id>`` with an ID from the list above):

    .. code-block:: bash

        ros2 topic echo /humans/faces/<face_id>/roi

3.  **Visualize in RViz:**

    Launch RViz and add the **Humans** display type (provided by ``hri_rviz``). This will overlay detected faces and bodies on your camera image.

Programming with ROS4HRI (Python)
---------------------------------

The ``pyhri`` library makes it easy to access HRI data in Python.

Here is a complete example of a node that detects if a person is looking at the robot.

.. code-block:: python

    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    import tf_transformations as t
    import numpy as np
    import hri

    class BodyOrientationListener(Node):

        def __init__(self, base_frame="camera_link", threshold=30):
            super().__init__('body_orientation_listener')
            
            # HRIListener automatically handles subscriptions to ROS4HRI topics
            self.hri_listener = hri.HRIListener("hri_listener_node")
            self.hri_listener.set_reference_frame(base_frame)

            self.threshold = threshold # Attention cone amplitude (degrees)

            self.create_timer(1.0, self.run) # Run at 1Hz

        def run(self):
            # Iterate over detected bodies
            for body_id, body in self.hri_listener.bodies.items():
                
                if not body.valid:
                    continue

                # Get transform from robot (base_frame) to body
                if not body.transform:
                    continue
                
                transform = body.transform
                
                # Extract translation
                trans = transform.transform.translation
                b2r_translation_x = trans.x
                b2r_translation_y = trans.y

                # Calculate angle
                b2r_xy_norm = np.linalg.norm([b2r_translation_x, b2r_translation_y])
                
                if b2r_xy_norm == 0:
                    continue

                angle = np.arccos(b2r_translation_x / b2r_xy_norm)

                # Check if body is facing the robot within threshold
                if angle < (self.threshold * np.pi / 180) and b2r_translation_x > 0:
                    self.get_logger().info(f"Body {body_id} is facing the robot!")

    if __name__ == "__main__":
        rclpy.init()
        node = BodyOrientationListener()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

**Explanation:**

1.  We initialize ``hri.HRIListener``, which aggregates data from ``/humans/`` topics.
2.  We access detected bodies via ``self.hri_listener.bodies``.
3.  For each body, we check its transformation relative to the robot.
4.  We calculate if the body is oriented towards the robot.

For more details, check the ``pyhri`` documentation: :ref:`pyhri <pyhri>`
