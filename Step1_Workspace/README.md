# Introduction 

1. Illustrate the components of a C++ workspace and demonstrates the build process shown below
![alt text](CPP_compilation.png)
2. Set up a Visual Studio workspace to build, run, and debug code. 
3. Employ colcon to manage builds. 

# Process 
1. Install VS Code 
     https://code.visualstudio.com/download
2. Set up VS Code for C++ in Linux
     https://code.visualstudio.com/docs/cpp/config-wsl
    While Windows is a perfectly acceptable environment for C++ in general, it is worthwhile to set up for Linux because most open source robotics applications are better supported in Linux. 
     2.1 Install WSL Extension for VS Code: https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl
     2.2 Install WSL in Windows: `wsl --install` in Powershell. Write down your username and password!! 
3. Set up Hello World Workspace in `Linux/Ubuntu/home/<username>` folder
     `mkdir Software/HelloWorld/src`
4. Write compile and run C++ code 
     4.1 `touch HelloWorld.cpp`
     4.2 Write main function that prints Hello World
     4.3 Compile code into executable, see [References][2]: `g++ -o HelloWorld src/HelloWorld.cpp`
     4.4 Run executable: `./HelloWorld`
     4.3 Celebrate! 
5. Set up VS Code configurations. See [Components/Configurations] 
     5.1 Test Run[Ctrl+F5] and Debug[F5] 
6. Set up version control. We'll use Git.
     6.1 Install Git 
         `sudo apt install git-all`
         `git --version`
     6.2 Create a repository for HelloWorld. In the HelloWorld folder.
         `git init`
     6.3 Commit code
          `git add .`
          `git commit -m "Say Hello World"`
     6.4 Push to GitHub (optional and homework) 

7. Link code to an External Library outside the workspace. We'll use Eigen: A C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
     7.1 Download Eigen to `~/Software`, i.e. outside this workspace
          `git clone https://gitlab.com/libeigen/eigen.git`
     7.2 Write a program `EigenAdd.cpp` to initialize a 2x2 matrix, assign numbers to 3 of its elements, and set a fourth to a sum of two elements. The code also prints the elements of the matrix when run.
     7.3 Build and execute `EigenAdd.cpp` by including the Eigen libraries, using the -I option
            `g++ -I ~/Software/eigen src/EigenAdd.cpp -o EigenAdd`
            `./EigenAdd`
8. Update VS Code configurations to include the Eigen external library 
     8.1 Add `"~/Software/eigen/**"` to `c_cpp_properties.json` so that Intellisense has knowlege of the headers. 
     8.2 Add 
# Components 
The workspace, aka `${workspaceFolder}`, consists of `.vscode`, `.git`, and `src` folders, as well as top-level files such as this Readme

## Software 
1. HelloWorld.cpp contains a main() function that prints Hello World. To build HelloWorld.cpp into and executable and run it

## Configurations
These are all defined in 
1. c_cpp_properties.json specifies the settings and properties for the C/C++ language extension in Visual Studio Code. It allows you to customize various aspects of your C/C++ development environment, such as include paths, compiler options, and IntelliSense settings. 
    - The ${workspaceFolder} represents this folder, the root folder of your workspace.  
2. tasks.json defines tasks that run when the user clicks Run or Run and Debug in VS Code. Run tasks can range from building your project, running scripts, launching external programs, to running custom commands. In this project, task.json defines the g++ command that is executed to build the code. 
3. launch.json outlines the process that runs in VS Code when Run/Debug (F5) is executed. It specifies which debugger to us, which executable to run, and the arguments for the executable. It also identifies any preLaunch tasks, such as compiling the code that gets debugged. preLaunch tasks may be defined in tasks.json.  


# References 
1. VS Code tutorials for getting started with C++ are detailed and up to date: https://code.visualstudio.com/docs/cpp/introvideos-cpp 
2. Mike Shah has great video that details the C++ compilation process: https://www.youtube.com/watch?v=ksJ9bdSX5Yo&t=1924s. I highly recommend this channel for learning and advancing C++ skills. 

 