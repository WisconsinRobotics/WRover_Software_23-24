# Setup: Development Environment using Docker

This document describes the setup process for a developing with a ROS environment Docker container.

Follow the instructions according to your platform:

* [Linux](#linux)
* [macOS](#macos)
* [Windows](#windows)

## Linux

### Tools

* [Docker Engine](https://docs.docker.com/engine)
* [Git](https://git-scm.com)
* [VS Code](https://code.visualstudio.com)

### Instructions

These instructions are written for Ubuntu, steps may vary if using a different distribution.

1. Install git: `sudo apt install git`
2. Install VS Code: https://code.visualstudio.com/docs/setup/linux
3. Install Docker Engine: https://docs.docker.com/engine/install/ubuntu
4. Set up SSH keys: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux
5. Clone the repository: `git clone git@github.com:WisconsinRobotics/WRoverPlayground.git`
6. Open the repository in VS Code
7. Clone git submodules: `git submodule update --init --recursive`
8. Install the VS Code [Dev Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
9. In the VS Code Command Palette (F1), run `Dev Containers: Open Folder in Container`

### Troubleshooting

**NOTE**: Please do not commit [`devcontainer.json`](./.devcontainer/devcontainer.json) if you need to modify it according to the following troubleshooting steps.

* After trying to run a GUI app, the following error is encountered: `Authorization required, but no authorization protocol specified`.
To fix this, try `xhost +localhost`, and see if the issue is resolved.
If not, run `xhost +`.
Since these command allow connections to your local X server, you may want to revert them with the corresponding `xhost -localhost` or `xhost -` after you are done using the devcontainer.
* After opening the devcontainer, a SELinux AVC denial is encountered.
The workaround is running `chcon -Rt svirt_sandbox_file_t /path/to/WRoverPlayground`.
Documented here: https://github.com/microsoft/vscode-remote-release/issues/1333
* After trying to run a GUI app, a SELinux AVC denial is encountered.
To fix this, add `"--security-opt=label=type:container_runtime_t"` to the `runArgs` in [`devcontainer.json`](./.devcontainer/devcontainer.json).
* If you are using Podman as a replacement for Docker, you will need to add `"--userns=keep-id"`  to the `runArgs` in [`devcontainer.json`](./.devcontainer/devcontainer.json).
Podman is not officially supported.

## macOS

### Tools

* [Docker Desktop](https://www.docker.com/products/docker-desktop)
* [Git](https://git-scm.com)
* [VS Code](https://code.visualstudio.com)
* [XQuartz](https://www.xquartz.org)

### Instructions

Apple Silicon Mac users should upgrade to macOS Ventura or newer.

1. Check if git is already installed by running `git --version` in the terminal
2. If git is not installed, and you are not prompted to install Xcode Command Line Tools, run `xcode-select --install`
3. Install VS Code: https://code.visualstudio.com
4. Install Docker Desktop: https://www.docker.com/products/docker-desktop
5. Apple Silicon users, follow the troubleshooting steps below to configure Docker for running ROS C++ programs
6. Install XQuartz: https://www.xquartz.org
7. Open XQuartz, and select XQuartz > Preferences
8. Under the "Security" tab, check ""Allow connections from network clients"
9. Restart your Mac and open XQuartz
10. In the terminal, run `xhost +`. You may need to rerun this every time you open XQuartz.
11. Set up SSH keys: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=mac
12. Clone the repository: `git clone git@github.com:WisconsinRobotics/WRoverPlayground.git`
13. Open the repository in VS Code
14. Clone git submodules: `git submodule update --init --recursive`
15. Install the VS Code [Dev Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
16. In the VS Code Command Palette (F1), run `Dev Containers: Open Folder in Container`

### Troubleshooting

* Apple Silicon Macs cannot run ROS C++ programs (requires macOS Ventura or newer)
    1. Open Docker Desktop
    2. Go to Settings (top right corner)
    3. Go to "Features in Development"
    4. Scroll down and check "Use Rosetta for x86/amd64 emulation on Apple Silicon"
    5. Press "Apply and Restart
* X11 forwarding instructions: https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088

## Windows

### Tools

* [Docker Desktop](https://www.docker.com/products/docker-desktop)
* [Git](https://git-scm.com)
* [VS Code](https://code.visualstudio.com)
* [WSL 2](https://learn.microsoft.com/en-us/windows/wsl/install)

### Instructions

These instructions are written for Windows 10 version 2004 or higher and Windows 11.

1. Open PowerShell or Command Prompt as Administrator
2. Run `wsl.exe --install` and restart if WSL is not already installed
3. If WSL is already installed, run `wsl.exe --update`
4. Check that WSL 2 is in use with `wsl.exe -l -v`
5. Install git: https://git-scm.com/downloads
6. Install VS Code: https://code.visualstudio.com
7. Install Docker Desktop: https://www.docker.com/products/docker-desktop
8. Set up SSH keys: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=windows
9. Open WSL by running `wsl.exe` in PowerShell or Command Prompt
10. In WSL, set up SSH keys: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux
11. In WSL, clone the repository: `git clone git@github.com:WisconsinRobotics/WRoverPlayground.git`
12. Open Docker Desktop
13. Open VS Code
14. Install the VS Code [WSL Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl) and [Dev Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
15. In the VS Code Command Palette (F1), run `WSL: Connect to WSL`
16. Open the repository
17. Clone git submodules: `git submodule update --init --recursive`
18. In the VS Code Command Palette (F1), run `Dev Containers: Open Folder in Container`

### Troubleshooting

* Unable to execute git push or pull in container: https://superuser.com/questions/1726204/get-agent-identities-ssh-agent-bind-hostkey-communication-with-agent-failed
