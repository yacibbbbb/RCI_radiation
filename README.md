# 🛰️ RCI_radiation - Radiation Simulation for Easy Robot Testing

[![Download RCI_radiation](https://img.shields.io/badge/Download%20RCI_radiation-blue?style=for-the-badge&logo=github)](https://github.com/yacibbbbb/RCI_radiation/releases)

## 🚀 What this is

RCI_radiation is a Windows-friendly simulation stack for radiation source and sensor testing. It helps you work with radiation maps, layered costmaps, and robot navigation in ROS 2 and Gazebo.

Use it if you want to:

- test a robot in a virtual space
- see how a radiation sensor responds
- build and check radiation maps
- try navigation logic before real-world use

## 📥 Download

1. Open the [RCI_radiation releases page](https://github.com/yacibbbbb/RCI_radiation/releases)
2. Find the latest release
3. Download the Windows package or setup file
4. Save the file to a folder you can find later

If the release page includes several files, choose the one for Windows. If there is a zip file, download that file first and extract it before running anything.

## 🖥️ Before you start

Use a Windows 10 or Windows 11 PC with:

- at least 8 GB of RAM
- 20 GB of free disk space
- a modern CPU
- a working internet connection

For best results, use:

- a dedicated GPU if you plan to run Gazebo smoothly
- a mouse, since simulation tools are easier to use with one
- a second monitor if you want to keep the simulation and docs open at the same time

## 🧭 How to install

### 1. Get the file

Go to the [release page](https://github.com/yacibbbbb/RCI_radiation/releases) and download the newest Windows file.

### 2. Open the download

If the file is a `.zip` file:

- right-click the file
- choose Extract All
- pick a folder
- wait for the files to unpack

If the file is a `.exe` installer:

- double-click the file
- follow the setup prompts
- choose the install folder if asked

### 3. Allow Windows to finish

Windows may ask for permission to run the app. Select the option that lets the file open.

### 4. Start the app

Open the installed folder or the extracted folder and launch the main program.

If the package includes a shortcut:

- double-click the shortcut to start the simulation

If it includes a launch file:

- open the file named for the main simulation or demo

## ▶️ First run

When the app opens, it may take a short time to load the map, robot model, and simulation tools.

A first run usually has these parts:

- a simulation window
- a robot or sensor model
- a radiation source or test scene
- a map view or costmap view
- a control panel for starting and stopping the run

If the window opens but stays blank for a moment, wait for the scene to finish loading.

## 🧪 What you can do with it

RCI_radiation is built for testing robot behavior in a radiation-aware scene. Typical tasks include:

- placing a radiation source in the map
- checking sensor output in different positions
- watching how radiation changes over distance
- testing path planning around risk areas
- viewing layered costmaps for safe movement
- running autonomous navigation checks in simulation

## 🗂️ Main parts of the system

### 🌡️ Radiation source model

This part simulates where radiation comes from and how it spreads in the map.

### 📡 Sensor model

This part shows how a detector may react when the robot moves near a source.

### 🗺️ Radiation map

This helps you view radiation levels across the scene.

### 🧱 Layered costmaps

These help the robot decide which areas are safer or less safe to cross.

### 🤖 Autonomous navigation

This helps test how a robot moves through the map without manual driving.

### 🎮 Gazebo simulation

Gazebo provides the virtual world where the robot, map, and sensors can be tested.

## 🧰 How to use it day to day

1. Start the app
2. Load the default scene or demo
3. Place or choose a radiation source
4. Watch the sensor reading
5. Move the robot to a new location
6. Check how the map and costmap change
7. Test the route the robot picks
8. Stop the run when you are done

If the package includes demo scenes, start with the default one. It gives the fastest path to a working test.

## 🛠️ Common setup tasks

### Change the simulation scene

Use the scene or world selector if the app provides one. Pick a simpler scene first if your PC runs slow.

### Restart a run

If the robot gets stuck or the scene stops responding:

- close the simulation window
- open it again
- reload the demo scene

### Reduce lag

If the app runs slowly:

- close other programs
- use a smaller map if one is available
- lower the graphic settings in Gazebo
- keep only one simulation window open

## 🔍 File types you may see

You may find these files in the download:

- `.exe` — Windows app or installer
- `.zip` — compressed folder
- `.bat` — script file for Windows
- `.launch` or `.launch.py` — start file for ROS 2
- `.world` — Gazebo world file
- `.yaml` — settings file for maps or costmaps

If you are not sure which file to open, start with the file that looks like the main launcher or installer.

## 🧭 Suggested first test

After you open the app, try this simple check:

1. Load the default map
2. Start the simulation
3. Move the robot near the radiation source
4. Watch the sensor value
5. Move away from the source
6. Confirm the reading changes

This gives you a quick view of whether the system is running well.

## 🧩 Troubleshooting

### The file will not open

- make sure the download finished
- right-click the file and try again
- check that Windows did not block it
- try running the file as the main user on the PC

### The app opens and closes

- extract the full zip folder before running
- do not move single files out of the folder
- try opening the app from the extracted folder
- restart the PC and try again

### Gazebo loads slowly

- close extra apps
- wait for the first load to finish
- use a machine with more memory if possible
- lower the scene detail if the option exists

### The robot does not move

- check that the simulation is running
- confirm the robot is not paused
- reload the scene
- try the default demo before a custom setup

### The display looks empty

- wait a little longer for assets to load
- check whether the world file opened
- restart the app
- use the default scene first

## 📌 Good habits for smooth use

- keep the release files in one folder
- use the newest release for your first run
- start with the default demo
- save custom scenes in a separate folder
- note which file you opened last time
- keep the release page bookmarked for updates

## 🧪 Best use cases

RCI_radiation fits well if you want to:

- train with a virtual radiation source
- test a detector before field work
- compare route choices near risk zones
- study layered map behavior
- build a safer navigation path in simulation

## 📎 Download again if needed

Use the official release page here: [https://github.com/yacibbbbb/RCI_radiation/releases](https://github.com/yacibbbbb/RCI_radiation/releases)

## 📝 What to expect after launch

Once the app starts, you should see a robot simulation setup that can load a map, show sensor output, and let you test motion in a radiation-aware space. The main goal is to give you a safe place to explore robot behavior before real hardware use