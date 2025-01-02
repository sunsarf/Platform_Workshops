# Introduction

Create a robotics workspace that can be used as a starting point for a variety of robotics platform. 
Create Python and C++ applications that communicate with each other through ROS2. 

# Watch and Learn 
https://youtu.be/P3tCqSTBhc0?si=25qKr9uhMl8uqlA7 

# Process
1. Write a compose.yaml file that pulls a Docker image with the Ubuntu 24.04 OS (Noble Numbat), ROS2 Jazzy Jalisco and its most widely used packages and GUIs, from the Docker registry and defines a persistent volume for the robotics workspace. For more about Docker, go to Part 2 of the platform workshop.[Containerize](https://github.com/sunsarf/Platform_Workshops/tree/main/Step2_Containerize)
2. Bring up a container using this Docker image by executing `docker compose up` from the folder containing the `compose.yaml` file. 
3. Enter the Docker container. 
    1. Click on the Docker extension, which looks like a whale, right click on `ros_jazzy`, select `Attach Visual Studio Code`, select the container in the search bar. VS Code should open a new window that is attached to the container. 
    2. Bring up a `Terminal>New Terminal` in the container and `cd \robotics_workspace`. `ls` to confirm that the files in this folder are listed in the `robotics_workspace` folder inside the Docker Container.
4. Source the ROS2 packages. 
    1. ROS2 is only available once its packages are sourced to the environment in the Docker container by executing `source /opt/ros/jazzy/setup.bash`
    2. ROS2 has been sourced successfully if you see a list of available commands when you execute `ros2`
5. Explore existing ROS2 packages 
    1. `ros2 pkgs list` lists all the pakages in this installation of ros2 while `ros2 pkgs executables` lists all the packages with their respective executables. 
    2. `ros2 run <package->executable>` runs a executable in ROS2
    3. Execute `ros2 run demo_nodes_cpp talker` and `ros2 run demo_nodes_cpp listener` in two seperate terminals within the ROS2 container/
    4. Now try  `ros2 run demo_nodes_cpp talker` and `ros2 run demo_nodes_py listener`. ROS2 enables the communication between C++ and Python processes!
    5. `rqt_graph` shows all the active nodes and the communications between them.  
    6. Exercise: Explore the `turtlesim` package.
6. Create a ROS2 Workspace. Learn about workspaces in Part 1 of the Platform Workshop series, [Workspace](https://github.com/sunsarf/Platform_Workshops/tree/main/Step1_Workspace)
    1. Navigate to `robotics_workspace`, the persistent volume we created in the `ros_jazzy` Docker container. 
    2. Create a `src` directory to make this a workspace. To learn more about workspaces go to 
    3. Colcon is a command-line interface(CLI) available in ROS2. It commands CMake and Python's setuptools. Try `colcon build` on the empty workspace to confirm whether it is available. You should see the `build`, `install`, and `log` folders. 
7. Add a Python package to the workspace
    1. Add a Python package to the `src` directory, by executing `ros2 pkg create hello_world_talker --build-type ament_python --dependencies rclpy` from the `src` directory. ROS2's `pkg` package creates the template for a Python package managed by the Ament build manager. It uses the ROS2 `rclpy` package as a dependency. 
    2. Review and modify `package.xml`, `setup.cfg`, and `setup.py`. 
    3. Navigate back to `robotics_workspace` and try `colcon build` again. Confirm that the `hello_world_talker` package gets built.
    4. Let's add some functionality to this package now. Add a file `talker_node.py` to `hello_world_talker/hello_world_talker`. Define a Talker Node class that says `Hello World` once every second and counts the number of times it says it. 
    5. Add an entry point for the package as the name of an executable, in the `entry_points` section of `setup.py`
    6. Navigate back to `robotics_workspace` and `colcon build` again. Source the newly built package to the terminal by running `source install/setup.bash`.
    7. Give the new package a whirl with `ros2 ros2 run hello_world_talker talk`. Observe the Hello World messages output every second. 
    8. Now modify the `talker_node` to publish the Hello World message to a `/chatter` topic, instead of the terminal. 
    View the data published to `/chatter` with `ros2 topic echo /chatter`, in a second terminal.
8. Add a C++ package to the workspace 
    1. Add a C++ package to the `src` directory, by executing `ros2 pkg create hello_world_translator --build-type ament_cmake --dependencies rclcpp` from the `src` directory. ROS2's `pkg` package creates the template for a Cmake package managed by the Ament build manager. It uses the ROS2 `rclcpp` package as a dependency. 
    2. Create a `translator_node` library declared in `include/hello_world_translator/translator_node.hpp` and implemented in `src/translator_node.cpp`
    3. Review and modify `package.xml`, CMakeLists.txt. Define the `TranslatorLib` library, `translate` executable, and include directories. 
    4. Navigate back `robotics_workspace` and `colcon build` to build the C++ and Python packages. Source the newly built packages to the terminal by running `source install/setup.bash`.
    5. Open 4 terminals up this time and source the ROS2 packages and these packages by executing `source /opt/ros/jazzy/setup.bash` and `source install/setup.bash`
    6. Run the following commands, one in each terminal, to see the communication between talker and translator, fascilitated by ROS2
        1. `ros2 run hello_world_talker talk` 
        2. `ros2 run hello_world_translator translate`
        3. `ros2 topic echo /chatter`
        4. `ros2 topic echo /charla`
    7. Launch the talk and translate applications with a launch file. This is typical for complex systems where multiple applications are launched at once. 
        1. Create a launch directory in the `robotics_workspace` with `mkdir launch`
        2. Write a launch description, `launch/talk_and_translate.py` that lists the executables to launch
        3. Update the `package.xml` files for the packages corresponding to these executables to include the launch dependency 
        4. Rebuild with `colcon build`
        5. Launch with `ros2 run launch/talk_and_translate.py`. 
        6. In a second terminal, execute `rqt_graph` to see the active nodes and topics. 
    8. Celebrate! 
 

# Components
1. `Compose.yaml` defines the `robotics` service. The `robotics` service is the ROS2 Jazzy Desktop middleware and applications running in an Ubuntu Noble environment.
2. `robotics_workspace` is a ROS2 workspace for all the packages we create in ROS2. Within this workspace, we create
    1. `hello_world_talker`, a Python package that publishes a Hello World message to a `/chatter` topic at 1Hz
    2. `hello_world_translator`, a Python package that subscribes to the  Hello World message to the `/chatter` topic and publishes translations to the `/charla` topic
3. `.vscode` contains configurations for VS Code, as described in [Workspace](https://github.com/sunsarf/Platform_Workshops/tree/main/Step1_Workspace)


# References
1. [Robotics Back End's ROS2 Tutorial](https://roboticsbackend.com/category/ros2/)
2. [C++ Publisher and Subscriber](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
3. [Creating Launch Files](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
