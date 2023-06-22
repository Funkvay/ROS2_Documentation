# ROS2_Documentation

Getting Started with ROS2

This guide is intended to help you install and set up ROS2, and get started with creating a Python and C++ packages in ROS2.

1. ### Download ROS2

    If you use Ubuntu 22.04: Install ROS2 Humble
    If you use ROS2 Ubuntu 20.04: Install ROS2 Foxy

2. ### Setup your Environment for ROS2

    Source the ROS2 environment setup file. Make sure to use the correct path based on the ROS2 distribution you installed (humble or foxy).

    ```
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```
    OR for Foxy
    ```
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    ```

3. ### Verify ROS2 Installation

    Check if ROS2 is installed correctly by running a simple demo:
    ```
    ros2 demo_nodes_cpp talker
    ```
    In another terminal, type:
    ```
    ros2 demo_nodes_cpp listener

    ```
    If the listener successfully communicates with the talker, then ROS2 has been fully installed.

4. ### Install Colcon Build Tool

    Colcon is a build tool for ROS2. Install it using the following commands:
    ```
    sudo apt install python3-colcon-common-extensions
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
    ```

5. ### Create a Workspace

    Create and build a new ROS2 workspace:
    ```
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    colcon build
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

    ```

    WARNING: Your workspace is where you develop your ROS2 packages. When you build your workspace, a `setup.bash` script is generated in the `install` directory of your workspace. This script contains information specific to the packages you are developing in your workspace.

    When you source your workspace’s `setup.bash`, it sets up:
    * Environment variables specific to your workspace.
    * Paths to the libraries and executables of the packages in your workspace.
    This ensures that your system knows where to find the packages you are developing, and you can run and use them seamlessly with the rest of the ROS2 ecosystem.

6. ### Create a Python Package

    Create a new Python package within your workspace:
    ```
    cd ~/ros2_ws/src
    ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
    cd ~/ros2_ws
    colcon build
    ```
    #### Note: If you are using ROS2 Humble and encounter an error when executing colcon build, you may need to downgrade the setuptools version. Do this by running:

    ```
    pip3 install setuptools==58.2.0
    colcon build
    ```

7. ### Create a C++ Package

    In your workspace `/ros2_ws/src` directory, run the following command:
    ```
    ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
    ```

    #### Note: rclcpp stands for ROS Client Library for C++.

8. ### Write a C++ Node and Run it

    Navigate to `/ros2_ws/src/my_cpp_pkg/src` and create a file named `my_first_node.cpp`.

    Make sure to `include rclcpp/rclcpp.hpp` in your .cpp file:
    ```
    #include rclcpp/rclcpp.hpp
    ```

    To run your node, first build it using:

    ```
    colcon build --packages-select my_cpp_pkg
    ```

    Then, execute the following command to run the node:

    ```
    ros2 run my_cpp_pkg cpp_node_executable_name
    ```
    For instance, if your CMake file contains:
    ```
    add_executable(cpp_node src/my_first_node.cpp)
    ament_target_dependencies(cpp_node rclcpp)
    ```
    Your executable will be named `cpp_node`.

9. ### Debug and Monitor Your Nodes with ros2 CLI

    After running your node with the following command:
    ```
    ros2 run my_cpp_pkg cpp_node_executable_name
    ```
    Open another terminal and use the command below to view active nodes:
    ```
    ros2 node list
    ```
    To run the same node multiple times, rename the nodes at runtime like this:
    ```
    ros2 run my_cpp_pkg cpp_node --ros-args --remap __node:=abc
    ```
    #### Note: the node now is called `abc`

10. ### Colcon

    When you modify the Python code, you need to build the package for changes to take effect. However, using the command below allows you to run your code without having to build it again.
    ```
    colcon build --packages-select my_py_pkg --symlink-install
    ```

    #### Note: This command creates symbolic links to your source files instead of copying them.

11. ### Rqt and rqt_graph

    RQT is a software framework that implements various GUI tools as plugins. Run RQT using:
    ```
    rqt
    ```
    Navigate to `Plugins->Introspection->Node` to see a graph of all the nodes that are currently running.

12. ### Understanding Turtlesim

    Turtlesim is a tool for teaching ROS and ROS packages. Install it using:
    ```
    sudo apt install ros-humble-turtlesim
    ```
    To run a simple example:
    ```
    ros2 run turtlesim turtlesim_node
    ```

13. ### Write a Python/C++ publisher

    For example, if you need a string example for the publisher, run:
    ```
    ros2 interface show example_interfaces/msg/String
    ```
    In Python, import it like this:
    ```
    from example_interfaces.msg import String
    ```
    In C++, include it like this:
    ```
    #include "example_interfaces/msg/string.hpp"
    ```

    Then, you can view active topics by running:

    ```
    ros2 topic list
    ```

    To see what a node is publishing:
    ```
    ros2 topic echo /topic_name
    ```
    For information about a node:
    ```
    ros2 node info /node_name
    ```

14. ### Publisher and Subscription

    For information about a topic:
    ```
    ros2 topic info /topic_name
    ```

    To monitor the rate at which data is published:
    ```
    ros2 topic hz /topic_name
    ```

    Publish from the terminal:

    ```
    ros2 topic pub -r 10 /topic_name example_interfaces/msg/String "{data: 'hello from terminal'}"
    ```

15. ### Mentioning

    Run turtlesim and observe how nodes communicate using rqt_graph:
    ```
    ros2 run turtlesim turtlesim_node
    ros2 run turtlesim turtle_teleop_key
    ```

16. ### ROS2 Services

    If you wrote a server in C++/Python, test it using:
    ```
    ros2 service list
    ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 5, b: 8"}
    ```
    
