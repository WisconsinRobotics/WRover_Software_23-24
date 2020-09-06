# Build Tools

Wisconsin Robotics uses a custom build script called `assemble.py`, which handles dependencies and delegates to catkin as necessary.
This document provides an overview of its functionality.

## Usage

First, you'll need to have Python 2.7.x and the `pip` and `virtualenv` tools installed.
The script is invoked from the command line as follows:

```shell
$ ./assemble.py [args...]
```

Running the script with no arguments gives a brief help message.
The subcommands are as follows:

* `$ ./assemble.py init`

  Performs initial setup for the workspace.
  This will prepare the catkin workspace as well as create a Python virtual env encapsulating the workspace.
  This only needs to be run once, at the very beginning.

* `$ ./assemble.py build [-o] [-f]`

  Builds the project.
  This will attempt to download and build each dependency, then build the workspace using `catkin_make`.
  Passing the `-o` flag enables offline mode, which disables downloading dependencies.
  Passing the `-f` flag enables force-update mode, which forces all dependencies to be deleted and re-downloaded regardless of whether there is actually a newer version available.

* `$ ./assemble.py clean`

  Deletes all downloaded dependencies.
  You probably don't want to use this unless you absolutely need to, for some reason.

## Project Structure

A project is defined by a project manifest file, which is the `project.json` in the root of this repository.
This file lists the dependencies of the project, describing how to download and build them.
The file is structured as follows:

```js
{
    "deps": [
        {
            "name": "dependency_name",
            "repo": "https://github.com/dependency/repo",
            "build": {
                "provider": "build_provider_name",
                // build provider config...
            }
        },
        // more dependencies...
    ]
}
```

Each dependency object should have a unique name.
The `repo` entry is only necessary if the dependency is to be built from a git repository, and can be omitted for dependencies obtained from other channels (e.g. the `apt` package manager).
The build provider describes how the dependency should be obtained and built; the existing ones are listed below:

### `cmake`

This provider uses `cmake` to build code in a git repository, then copies the resulting artifacts to the `/lib` subdirectory of dependent ROS packages.
The build provider configuration is as follows:

```js
{
    "provider": "cmake",
    "build_dir": "cpp", // the subdir where cmake should be invoked
    "artifacts": [ // list of build artifacts
        "cpp/src/libJoystickLibrary.a"
    ],
    "dependents": [ // list of dependent packages to copy artifacts to
        "drive",
        "arm",
        "science"
    ]
}
```

### `pip`

This provider uses `pip` to install dependencies of a Python project in a git repository.
Dependencies should be defined in a `requirements.txt` file in the root of the git repository.
If no requirements file is discovered, then it is assumed that there are no pip dependencies.
This build provider does not need a configuration.

Note that this is not how global pip dependencies are defined!
Global pip dependencies should be declared in a `requirements.txt` file in the root of the software system (i.e. this) repository.

### `apt`

This provider simply uses the `apt` package manager to install a dependency.
The build provider configuration is as follows:

```js
{
    "provider": "apt",
    "package": "ros-melodic-moveit" // the name of the package to install
}
```

These dependencies should be used with care, since they install the dependency system-wide rather than just in the catkin workspace!

## Other Notes

* It is generally recommended to use offline mode when possible to reduce build time; checking for updates can take a while.
