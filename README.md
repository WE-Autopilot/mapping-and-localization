# Mapping and Localization ROS2 Packages

## Table of Contents

1. [Requirements](#1-requirements)

   1. [Source ROS2 in each new terminal](#11-source-ros2-in-each-new-terminal)
   2. [Optional automatic ROS2 sourcing](#12-optional-automatic-ros2-sourcing)

2. [Setting Up a Local Workspace](#2-setting-up-a-local-workspace)

   1. [Create the workspace structure](#21-create-the-workspace-structure)
   2. [Clone this repository into src](#22-clone-this-repository-into-src)
   3. [Building the workspace](#23-building-the-workspace)
   4. [Source the workspace after building](#24-source-the-workspace-after-building)
   5. [Optional automatic workspace sourcing](#25-optional-automatic-workspace-sourcing)

3. [Repository Structure](#3-repository-structure)

4. [Working With the Two Packages](#4-working-with-the-two-packages)

   1. [C plus plus Package](#41-c-plus-plus-package)
   2. [Python Package](#42-python-package)

5. [Running Nodes](#5-running-nodes)

6. [Development Workflow and Rebuild Rules](#6-development-workflow-and-rebuild-rules)

   1. [When to Rebuild](#61-when-to-rebuild)
   2. [Recommended Developer Workflow](#62-recommended-developer-workflow)

7. [Notes for Developers](#7-notes-for-developers)

---

# 1. Requirements

Before working with this repository, ensure that:

* ROS2 Jazzy is installed
* colcon is installed
* You are using a Linux environment, for example Ubuntu inside UTM

This README assumes ROS2 is installed under:

```
/opt/ros/jazzy
```

---

## 1.1 Source ROS2 in each new terminal

ROS2 is not active automatically when a terminal is opened. Before using colcon or ros2 commands, run:

```bash
source /opt/ros/jazzy/setup.bash
```

This must be done in every new terminal unless you make it automatic.

---

## 1.2 Optional automatic ROS2 sourcing

If you want ROS2 to always be available:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

You still need to source your workspace separately after building it.

---

# 2. Setting Up a Local Workspace

All ROS2 development must happen inside a workspace that contains:

* src for packages
* build created by colcon
* install created by colcon
* log created by colcon

Your repository must go inside src.

---

## 2.1 Create the workspace structure

```bash
mkdir -p ~/mapping_ws/src
cd ~/mapping_ws/src
```

---

## 2.2 Clone this repository into src

```bash
cd ~/mapping_ws/src
git clone git@github.com:WE-Autopilot/mapping-and-localization.git
```

Resulting structure:

```
mapping_ws
  src
    mapping_and_localization
      mapping_and_localization_cpp
      mapping_localization_python
```

---

## 2.3 Building the workspace

### What building does

Running:

```bash
colcon build
```

makes colcon:

1. Compile C plus plus code
2. Install binaries into install
3. Register Python entry points
4. Generate environment setup scripts
5. Enable ros2 run to find the nodes

You must build when:

* C plus plus code changes
* package.xml changes
* CMakeLists.txt changes
* setup.cfg entry points change

### Build the workspace

```bash
cd ~/mapping_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Symlink mode is recommended for development.

---

## 2.4 Source the workspace after building

To use your nodes:

```bash
source ~/mapping_ws/install/setup.bash
```

You must source the workspace:

* In new terminals
* After every colcon build
* Before running ros2 commands

Python code works as long as you have sourced the workspace once in that terminal.

C plus plus requires sourcing after each rebuild.

---

## 2.5 Optional automatic workspace sourcing

If you use this workspace every day:

```bash
echo "source ~/mapping_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Make sure the order in your bashrc is:

```bash
source /opt/ros/jazzy/setup.bash
source ~/mapping_ws/install/setup.bash
```

---

# 3. Repository Structure

```
mapping_and_localization
  mapping_and_localization_cpp
    CMakeLists.txt
    package.xml
    src
      <cpp_nodes>.cpp

  mapping_localization_python
    package.xml
    setup.cfg
    setup.py
    mapping_localization_python
      <python_nodes>.py

  .gitignore
  README.md
```

Only the two folders with package.xml are ROS2 packages.

---

# 4. Working With the Two Packages

## 4.1 C plus plus Package: mapping_and_localization_cpp

### Adding a node

In CMakeLists.txt:

```cmake
add_executable(localization_node src/localization_node.cpp)

ament_target_dependencies(localization_node
  rclcpp
)

install(TARGETS
  localization_node
  DESTINATION lib/${PROJECT_NAME}
)
```

### Adding dependencies

In package.xml:

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

In CMakeLists.txt:

```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```

---

## 4.2 Python Package: mapping_localization_python

### Adding a node

In setup.cfg:

```ini
[options.entry_points]
console_scripts =
  pose_node = mapping_localization_python.pose_node:main
```

### Dependencies

In package.xml:

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
```

Python files live inside:

```
mapping_localization_python/mapping_localization_python/
```

---

# 5. Running Nodes

You must source the workspace first:

```bash
source ~/mapping_ws/install/setup.bash
```

Run a C plus plus node:

```bash
ros2 run mapping_and_localization_cpp localization_node
```

Run a Python node:

```bash
ros2 run mapping_localization_python pose_node
```

---

# 6. Development Workflow and Rebuild Rules

## 6.1 When to Rebuild

| What changed                      | Need `colcon build --symlink-install` | Reason                            |
| --------------------------------- | ------------------------------------- | --------------------------------- |
| Python file (`.py`)               | No                                    | Symlink points to source file     |
| Launch, YAML, configuration       | No                                    | Symlinks update instantly         |
| C plus plus file (`.cpp`, `.hpp`) | Yes                                   | Must recompile                    |
| package.xml                       | Yes                                   | Metadata changed                  |
| CMakeLists.txt                    | Yes                                   | Build system instructions changed |
| New Python node in setup.cfg      | Yes                                   | Must regenerate entry point       |
| Editing setup.cfg text only       | Usually no                            | Not used at runtime               |
| Adding a new package              | Yes                                   | Build system must detect it       |

---

## 6.2 Recommended Developer Workflow

### Python workflow

1. Edit the Python file
2. Source the workspace once in any terminal
3. Run the node

No rebuild and sourcing workspace needed unless adding new entry points.

### C plus plus workflow

1. Edit the C plus plus file
2. Run:

   ```bash
   colcon build --symlink-install
   ```
3. Then source again:

   ```bash
   source install/setup.bash
   ```
4. Run the node

Rebuild required for all C plus plus changes.

### After modifying package.xml, CMakeLists.txt, or setup.cfg

1. Rebuild with symlink
2. Source workspace
3. Run the node

---

# 7. Notes for Developers

* Only folders with package.xml are ROS2 packages
* build, install, log and vscode should not be committed
* Always run colcon build from the workspace root, never inside a package
* Never run CMake manually


