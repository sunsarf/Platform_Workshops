# Introduction 

Run Python application in a virtual environment defined by Docker. 

# Process 
1. Install Docker
    1. Install the VS Code Docker Extension, `ms-azuretools.vscode-docker`
    2. Check if docker is already installed with `sudo docker run hello-world`
    3. If not, [Install the Docker Engine](https://docs.docker.com/engine/install/ubuntu/) on Ubuntu, using the WSL Terminal
    4. Confirm that you see "Hello from Docker! This message shows that your installation appears to be working correctly." when you rerun `sudo docker run hello-world`
2. [Containerize a Python Application](https://docs.docker.com/language/python/containerize/)
3. [Add a database for Python Application](https://docs.docker.com/language/python/containerize/)
4. [Docker Commands](https://docs.docker.com/get-started/docker_cheatsheet.pdf)
4. Remotely connecting to a Docker Container with VS Code 

# Components of python-docker
1. `Dockerfile` defines the instructions to assemble a Docker Image. This image contains defines file system parameters and the command to run the application. 
2. `compose.yaml` define and manage multi-container applications in a single YAML file
3. `.dockerignore` lists the dev files that the application doesn't need, so the container only includes that files necessary for the application to run

# Useful extensions 
1. Python, ms-python.python


# References 
1. [Why use Docker?](https://docs.docker.com/guides/docker-overview/)
2. [Why use Compose?](https://docs.docker.com/compose/intro/features-uses/)