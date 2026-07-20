"""ROS 2 navigation bridge for cat_follow.

These modules depend on ``rclpy`` and the ROS 2 message packages, which are
only present on the ROCK 4D deployment (sourced from ``/opt/ros/jazzy``).  They
are intentionally import-guarded so the rest of ``cat_follow`` (and the test
suite) import cleanly on machines without ROS installed.
"""
