cmake_minimum_required(VERSION 3.8)
project(cat_follow_bringup)

find_package(ament_cmake REQUIRED)

# Install shared resources so they resolve via get_package_share_directory().
install(
  DIRECTORY launch config urdf maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
