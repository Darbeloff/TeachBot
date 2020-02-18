# TeachBot

## First Setup Only
The following is adapted from http://sdk.rethinkrobotics.com/intera/Workstation_Setup and https://wiki.ros.org/kinetic/Installation/Ubuntu. It runs exclusively on Ubuntu 16.04. If you run this on newer versions of Ubuntu, you will have to update `install.sh` for the latest ROS distro.

### Clone Development Workspace
If you change the directory, make sure you adjust all following commands.
```
$ cd
$ sudo apt -y install git
$ git clone https://github.com/Darbeloff/TeachBot.git
```

### Install Using the Installation Script
Run the following commands in terminal:
```
$ cd ~/TeachBot
$ ./install.sh
```

Alternatively, if you do not run `install.sh` because you already installed the dependencies previously, and now you just want to prepare a freshly cloned version of this repository to work, run the following commands in terminal:
```
$ cd ~/TeachBot
$ ./setup.sh
```
That setup script only merges the Intera resources and installs NPM directories.

### Set Up Networking
Connect to the robot via the ethernet port on the outside of the Controller.
![Source: http://sdk.rethinkrobotics.com/intera/Workstation_Setup](http://sdk.rethinkrobotics.com/intera/a/images/e/ec/Ethernet_Port.png)

Open Ubuntu's "Settings" tool, and navigate to the "Network" tab. Then, click the gear icon in the "Wired" section.
![Click the gear icon in Network Settings](/images/network_settings.png?raw=true)

Navigate to the IPv4 tab and select "Link-Local Only." Then, click "Apply."
![Enable Link-Local Only](/images/wired_settings.png?raw=true)

You will not be able to use that ethernet port to access the internet until you revert the Method to "Automatic (DHCP)." You may now close the Settings window.

### Verify Environment
```
$ cd ~/TeachBot/robot
$ ./intera.sh
```

You should now be in a new shell titled `intera - http://tom.local:11311` where `tom` is the name of your robot. A useful command for viewing and validating your ROS environment setup is:
```
$ env | grep ROS
```

The important fields at this point:
`ROS_MASTER_URI` - This should now contain your robot's hostname.
`ROS_HOSTNAME` - This should contain your PC's hostname.
Try to get rostopic list from robot by typing following command:
```
$ rostopic list
```

You should see a the rostopic list from the command line output similar as following:
```
/array_topic
/audio_duration
/calibration_command/cancel
/calibration_command/feedback
/calibration_command/goal
/calibration_command/result
/calibration_command/status
/camera_field_calibration/cancel
/camera_field_calibration/feedback
/camera_field_calibration/goal
/camera_field_calibration/result
/camera_field_calibration/status
/cmd2browser
/cmd2shell
/collision/right/collision_detection
/collision/right/debug
/dev_topic
/diagnostics
/engine/task_state
/errors
/head_navigator_button
/intera/endpoint_ids
/io/comms/command
/io/comms/config
/io/comms/io/command
/io/comms/io/config
/io/comms/io/state
/io/comms/state
/io/end_effector/command
/io/end_effector/config
/io/end_effector/state
/io/internal_camera/command
...
```

## To Launch
Navigate to the `TeachBot/robot` directory.
```
$ cd ~/TeachBot/robot
```

Initialize SDK environment
```
$ ./intera.sh
```

Launch the websocket and start Node.js
```
$ roslaunch ../browser/websocket.launch & node ../browser/www.js &
```

Run the TeachBot ROS Node
```
$ rosrun sawyer teachbot.py
```

Run the Button Box ROS Node
```
$ rosrun button_box button.py
```

## Load Modules in Firefox
Open Firefox and go to the url https://localhost:8000

### If Firefox alerts you that your connection is not secure
Click "Advanced"

![Click "Advanced"](/images/insecure_connection.png?raw=true)

Click "Add Exception"

![Click "Add Exception"](/images/add_exception.png?raw=true)

Click "Confirm Security Exception"

![Click "Confirm Security Exception"](/images/confirm_exception.png?raw=true)

At this point, the module should begin, but there should be a pop-up window that reads "Error connecting to websocket server" like this:

![Navigate to localhost:9090 and repeat the security exception process.](/images/connection_error.png?raw=true)

Then, go to the url https://localhost:9090 and repeat the security exception process. After, https://localhost:9090 should simply read `Can "Upgrade" only to "WebSocket".` Navigate back to https://localhost:8000.
### EndIf

You have successfully launched the teaching module!

## Repository Overview
```
.
|
+-- browser                             Files related to the HTML/CSS/JS running in-browser.
|   +-- public                          Front-End Browser Content.
|       +-- audio                       TeachBot speech audio files.
|           +-- module#                 Directories containing subdirectories of audio files, indexed by module.
|               +-- <section_name>/     Directories containing audio files indexed by line number.
|           +-- make_speech.py          Script to generate speech audio files from text scripts.
|       +-- css/                        Style sheets for the browser HTML
|       +-- html/                       Web pages to display in browser.
|       +-- images/                     Images displayed in browser by TeachBot.
|       +-- js                          JavaScript module control scripts and utilities.
|           +-- json/                   JavaScript Object Notation (JSON) files containing the instructions for each module.
|           +-- utils/                  Utility functions for browser.
|           +-- Module.js               Module object class definition.
|           +-- #.js                    Module control scripts for browser indexed by module.
|       +-- text                        The text of what TeachBot says.
|           +-- module#                 Directories containing .txt files of what TeachBot says, indexed by module.
|               +-- <section_name>.txt  Text file containing all lines spoken by TeachBot in a given section.
|       +-- videos/                     Videos played in browser by TeachBot.
|   +-- routes/                         Node.js routes used by app.js.
|   +-- views/                          App templates.
|   +-- app.js                          Main configuration file for TeachBot browser.
|   +-- CMakeLists.txt                  Node CMakeLists.
|   +-- package-lock.json               NPM install utility.
|   +-- package.json                    NPM install utility.
|   +-- package.xml                     Node package manifest.
|   +-- websocket.launch                ROS launch file for web module.
|   +-- www.js                          Starts Node.js server.
|
+-- images/                             Images for the README.
|
+-- robot                               Python ROS library responsible for communicating with the robot.
|   +-- src                             Source code.
|       +-- sawyer                      TeachBot ROS package for the RethinkRobotics Sawyer.
|           +-- Learner_Responses/      Data collected from user subject tests.
|           +-- msg/                    Custom message files for information transfer to and from the browser.
|           +-- src/                    Python source code.
|               +-- teachbot.py         ROS listener controlling the robot and receiving commands from the browser.
|               +-- <other>.py          Utility classes and functions used by module control scripts.
|           +-- CMakeLists.txt          ROS package CMakeLists.
|           +-- package.xml             ROS package manifest.
|           +-- setup.py                ROS package setup script.
|   +-- sslcert/                        SSL certificate files required for setting up HTTPS connection.
|   +-- logo.png                        TeachBot logo to be displayed on Sawyer head display.
|   +-- safety1.mp3                     Audio file to be played when TeachBot limb exits safety zone.
|   +-- safety2.mp3                     Audio file to be played when TeachBot limb resets to within the safety zone.
|
+-- edit_intera.py                      Used by install.sh. Copies and edits intera.sh into TeachBot/robot.
+-- install.sh                          Installation script.
+-- LICENSE                             BSD 3-Clause software license.
+-- README.md                           This document.
+-- setup.sh                            Used by install.sh. Merges Intera resources and installs NPM dependencies.
```
The project is organized in two segments:
1) JavaScript Node application, responsible for coordinating the module and displaying content in the browser, and
2) Python ROS library, responsible for communicating with the robot.

### JavaScript Node Application
The JavaScript scripts controlling the browser are located [here](/browser/public/js) and are indexed by module (e.g. `1.js`, `2.js`, and so on). The layout is simple:

1. Construct Module object.
2. Initialize main() function, which starts the Module.

This should be the last script included in the HTML.

The crux of these files is `Module.js`, the constructor of the Module object. Upon construction, a Module sets up ROS communication with the robot, prepares the browser for a Module, and loads all of the resources needed to run the module including the text, audio, and JSON files. Once everything is loaded, it runs the `main()` function provided by the calling script. The `main()` function should always conclude with a call to `Module.start()`, which begins recursively iterating through the instructions in the JSON file.

The server running the browser half of the application can be started using the command described above:
```
$ roslaunch ../browser/websocket.launch & node ../browser/www.js &
```

### Python ROS Library
All Python ROS packages are located in [the robot source directory](/robot/src/).

With the exception of `arduino_files` and the files imported from other repositories, each package is named after the robot it commands (e.g. `sawyer` should be used when connected to the RethinkRobotics Sawyer cobot). In the source directory of each package, there should be a `teachbot.py` script that runs the central listener, receiving instructions from the JavaScript Module over ROS, forwarding those commands to the robot over Ethernet, and communicating data from the robot back to the JavaScript Module.

The main `teachbot.py` scripts are run using the command described above:
```
$ # This is just the format of the command. Do not actually enter <name_of_cobot>.
$ rosrun <name_of_cobot> teachbot.py
```
