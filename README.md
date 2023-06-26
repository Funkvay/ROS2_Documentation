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

8. ### Write a C++ Node, Configure CMakeLists.txt, and Run the Node
    #### a. Create C++ Source File
    
    First, navigate to `/ros2_ws/src/my_cpp_pkg/src` and create a file named `my_first_node.cpp`.

    Include the `rclcpp` library in your `.cpp` file, as it is necessary for writing ROS2 nodes in C++:

    ```#include rclcpp/rclcpp.hpp```

    #### b. Configure CMakeLists.txt and package.xml

    To build the node, you need to configure the `CMakeLists.txt` file in your package directory `/ros2_ws/src/my_cpp_pkg`.

    Open `CMakeLists.txt` and ensure it includes the following lines:

    ```
    # Set minimum CMake version
    cmake_minimum_required(VERSION 3.8)

    # Define project name
    project(my_cpp_pkg)

    # Set compiler options
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # Find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(example_interfaces REQUIRED) # for example_interfaces etc

    # Create an executable
    add_executable(cpp_node src/my_first_node.cpp)
    ament_target_dependencies(cpp_node rclcpp)

    # Install targets
    install(TARGETS
    cpp_node
    DESTINATION lib/${PROJECT_NAME}
    )

    # Packaging
    ament_package()
    ```

    #### package.xml

    here you just need to add 
    ```
    <depend>rclcpp</depend>
    <depend>example_interfaces</depend>
    ```
    and other dependencies

    #### c. Build the Node

    To build your node, navigate to the root of your workspace and execute the following command:

    ``` colcon build --packages-select my_cpp_pkg```

    This will build the package `my_cpp_pkg`.

    #### d. Run the Node
    After building the package, you can run the node using the following command:
    ```ros2 run my_cpp_pkg cpp_node```

    Here, my_cpp_pkg is the name of the package, and cpp_node is the name of the executable we specified in CMakeLists.txt.
    

9. ### C++ node example

#### Including Necessary Headers

```
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <functional>
```

`#include "rclcpp/rclcpp.hpp"` includes the necessary header file for using the ROS2 C++ client library.

#### Defining a Custom Node Class

```
class MyNode: public rclcpp::Node
{
public:
    MyNode()
    : Node("cpp_test"), counter(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello cpp node");
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyNode::timerCallBack, this)); 
    }

private:
    int counter;

    void timerCallBack()
    {
        std::string text = "Hello " + std::to_string(counter++);
        RCLCPP_INFO(this->get_logger(), text.c_str());
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

```

* The constructor MyNode() initializes the base class rclcpp::Node with the name "cpp_test" and initializes counter to 0. It also prints "Hello cpp node" to the console using RCLCPP_INFO.
* `timer_` is assigned a timer instance using `this->create_wall_timer`. The timer fires every second and calls the `timerCallBack` method.
* `timerCallBack` is a member function that is called every time the timer fires. It increments counter and prints a message to the console.
* `rclcpp::TimerBase::SharedPtr timer_;` declares `timer_` as a shared pointer of type `rclcpp::TimerBase`. This is used to store the timer instance.

#### Attention !
#### rclcpp::TimerBase::SharedPtr vs rclcpp::TimerBase
In ROS2, timers are typically managed through shared pointers rather than raw pointers or objects.

Why not to use use rclcpp::TimerBase:
* Handling the timer directly may lead to manual memory management, and the potential for memory leaks or undefined behavior.

As a best practice in ROS2, use `rclcpp::TimerBase::SharedPtr` for managing timers, as it simplifies memory management and improves safety.

#### The main Function
```
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MyNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

```

* `rclcpp::init(argc, argv);` initializes the ROS2 communication layers.
* `rclcpp::spin(node);` causes the ROS2 node to process callbacks (like the timer callback) until it is shut down.
* `rclcpp::shutdown();` cleans up the ROS2 communication resources.



10. ### Debug and Monitor Your Nodes with ros2 CLI

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

11. ### Colcon

    When you modify the Python code, you need to build the package for changes to take effect. However, using the command below allows you to run your code without having to build it again.
    ```
    colcon build --packages-select my_py_pkg --symlink-install
    ```

    #### Note: This command creates symbolic links to your source files instead of copying them.

12. ### Rqt and rqt_graph

    RQT is a software framework that implements various GUI tools as plugins. Run RQT using:
    ```
    rqt
    ```
    Navigate to `Plugins->Introspection->Node` to see a graph of all the nodes that are currently running.

13. ### Understanding Turtlesim

    Turtlesim is a tool for teaching ROS and ROS packages. Install it using:
    ```
    sudo apt install ros-humble-turtlesim
    ```
    To run a simple example:
    ```
    ros2 run turtlesim turtlesim_node
    ```

14. ### Write a Python/C++ publisher

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

15. ### Publisher and Subscription

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

16. ### Mentioning

    Run turtlesim and observe how nodes communicate using rqt_graph:
    ```
    ros2 run turtlesim turtlesim_node
    ros2 run turtlesim turtle_teleop_key
    ```

17. ### ROS2 Services

    If you wrote a server in C++/Python, test it using:
    ```
    ros2 service list
    ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 5, b: 8"}
    ```
