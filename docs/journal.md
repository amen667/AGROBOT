# AGROBOT Engineering Journal

## Day 1 - Project Initialization

### Objective
Start the AGROBOT project and organize it like a real engineering project.

### What I did
- Changed the project idea from PHARMABOT to AGROBOT.
- Defined the goal: mini-greenhouse inspection robot.
- Identified the main hardware.
- Created the AGROBOT folder structure.
- Installed Git.
- Configured Git user name and email.
- Created the first local Git repository.
- Created the GitHub repository.
- Pushed the first version online.
- Added the project README.

### What I learned
- Git saves project versions locally.
- GitHub stores the project online.
- A README explains the project.
- A clean folder structure makes the project easier to manage.

### Problems
- Windows created `.txt` files instead of `.md`.
- Some folders were empty, so Git did not track them.

### Solutions
- Renamed `.md.txt` files to `.md`.
- Added `.gitkeep` files to keep empty folders.

### Next Objective
Prepare Ubuntu installation and start learning ROS2.
Day 2 - Ubuntu and ROS2 Setup
Objective

Install Ubuntu and prepare the ROS2 development environment for AGROBOT.

What I did
Installed Ubuntu 24.04.4 LTS alongside Windows.
Completed the first Ubuntu setup.
Installed basic development tools.
Installed Git on Ubuntu.
Cloned the AGROBOT GitHub repository into Ubuntu.
Installed Visual Studio Code on Ubuntu.
Installed ROS2 Jazzy.
Tested ROS2 communication using the talker and listener demo nodes.
Created the ROS2 workspace inside the AGROBOT project.
Created the first ROS2 package: agrobot_basics.
Created the first custom AGROBOT ROS2 node: status_publisher.
Published messages on the topic /agrobot/status.
Verified the messages using ros2 topic echo.
Committed and pushed the first ROS2 node to GitHub.
What I learned
Ubuntu and Windows are separate systems in dual boot.
GitHub lets me recover the project on Ubuntu using git clone.
ROS2 projects are organized into workspaces, packages, and nodes.
A node is a small program that does one job.
A topic is a communication channel between nodes.
A publisher sends messages to a topic.
A subscriber reads messages from a topic.
colcon build builds the ROS2 workspace.
source install/setup.bash makes the terminal recognize the workspace.
Problems
GitHub did not accept normal password authentication from terminal.
The first run of status_publisher showed “No executable found”.
Some generated ROS2 folders appeared in git status.
Solutions
Installed GitHub CLI and logged in using gh auth login.
Fixed setup.py and rebuilt the package.
Added .gitignore to ignore build, install, and log folders.
Result

The AGROBOT ROS2 workspace is ready, and the first custom ROS2 node successfully publishes status messages.

Next Objective

Connect the YDLIDAR X4 Pro and publish LiDAR scan data on /scan.

After saving it, run:

git add docs/journal.md
git commit -m "Add Ubuntu and ROS2 setup journal"
git push
## Day 3 - YDLIDAR ROS2 Setup

### Objective
Connect the YDLIDAR X4 Pro to ROS2 and publish laser scan data.

### What I did
- Connected the YDLIDAR through USB.
- Installed the YDLIDAR SDK.
- Installed the YDLIDAR ROS2 driver.
- Fixed the configuration using the X4 settings.
- Changed the LiDAR to single-channel mode.
- Verified that `/scan` is published in ROS2.
- Tested `/scan` using `ros2 topic echo /scan --once`.
- Verified scan frequency using `ros2 topic hz /scan`.

### Result
The LiDAR successfully publishes LaserScan data on `/scan` at around 11 Hz.

### Problem
RViz did not show the laser points clearly even though `/scan` was active.

### Next Objective
Use `/scan` with SLAM Toolbox to start building a map.