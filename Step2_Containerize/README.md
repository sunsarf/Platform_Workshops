# Introduction 

Run Python application in a virtual environment defined by Docker. 

# Watch and Learn 
https://youtu.be/He57W-Kxeu0?si=E7K1rcasNkwK3vGS 

# Process 
1. Install Docker
    1. Install the VS Code Docker Extension, `ms-azuretools.vscode-docker`
    2. Check if docker is already installed with `sudo docker run hello-world`
    3. If not, [Install the Docker Engine](https://docs.docker.com/engine/install/ubuntu/) on Ubuntu, using the WSL Terminal
    4. Confirm that you see "Hello from Docker! This message shows that your installation appears to be working correctly." when you rerun `sudo docker run hello-world`
2. [Containerize a Python Application](https://docs.docker.com/language/python/containerize/)
    1. If you have the Docker extension installed, you can use it to add Docker files to the workspace by searching "Add Docker Files" in the Command Palette. The Command Palette is a searchable registry of commands for all the installed extensions. Access it by typing `Ctrl+Shift+P`. Modify the added files based on the instructions above. 
3. [Add a database for Python Application](https://docs.docker.com/language/python/containerize/)
4. [Docker Commands](https://docs.docker.com/get-started/docker_cheatsheet.pdf)
5. Use the Docker extension to remotely connect to a Docker Container with VS Code 
    1. First try executing the container in the WSL Terminal with `docker exec -it <container_name> /bin/bash`. You'll see the command prompt change to `appuser@<ContainerUUID>:/app$`. Replace <container_name> with the name corresponding to the python-docker-server image. The command prompt starts at `/app` because this is defined as the working directory in the Dockerfile. `exit` to return to the WSL terminal.
    2. The procedure above works to quickly troubleshoot issues inside a docker container. However, when the same Docker image is routinely used for development, it is useful to see the container file system and terminal in VS code. VS Code allows one to connect to Docker containers and operate within them. 
    3. Click on the Docker Extension, which looks like a whale, and right click on the `python-docker-1` container in the Containers tab. 


# Components of python-docker
1. `Dockerfile` defines the instructions to assemble a Docker Image. This image contains defines file system parameters and the command to run the application. 
2. `compose.yaml` define and manage multi-container applications in a single YAML file
3. `.dockerignore` lists the dev files that the application doesn't need, so the container only includes that files necessary for the application to run

# Useful extensions 
1. Python, ms-python.python


# References 
1. [Why use Docker?](https://docs.docker.com/guides/docker-overview/)
2. [Why use Compose?](https://docs.docker.com/compose/intro/features-uses/)
3. [Working with Docker and VSCode](https://www.youtube.com/watch?v=wUUmRbXiIOo&t=1058s), an in-depth tutorial that demonstrates the capabilities of both Docker and VSCode.
