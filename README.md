# 2024-Java-Testing
This repository contains examples we have worked through during the Summer of 2024. All examples have been tested and run on Holy Canoli and/or the 2024 competition robot for the Cresendo game.

## Initial Setup

Below are the minimal steps to get up and running with running Java on our FRC Robots and testbeds using our example code.

### Install FRC Game Tools

To be able to control and test with the RoboRIO, we will need to have the following software installed:
  - FRC Game Tools
    - LabVIEW Update
    - FRC Driver Station
    - FRC RoboRIO Imaging Tool and Images

For detailed documentation on how to install, FIRST has great documentation [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html).

### Installing VSCode with WPILib Command Palette

To develop Java code for our robots, we are utilizing a special version of VSCode that has been customized to make FRC Robot development easier.

For detailed documentation on how to install, FIRST has great documentation [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).
  - NOTE: FIRST may not have updated their docs to point to the latest version of VSCode with WPILib, to check for the latest version, you can view it on their Github release page [here](https://github.com/wpilibsuite/vscode-wpilib/releases).

### Preparing Your Robot

If you are using hardware (RoboRIO and Radio) that has already been configured, you can skip this section. Otherwise, if your RoboRIO and Radio are new, you will need to image and program them respectively.

For detailed documentation on how to this, FIRST has great documentation, below are the links to the documentation for each:
  - [Imaging RoboRIO](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/roborio2-imaging.html)
  - [Programming Radio](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/radio-programming.html)

At this point you should have all the software tools and necessary hardware equipment setup to be able to start working through these examples.

### Clone this repository

To use and work with our examples, you will need to clone this repository from Github. We use [GIT](https://www.git-scm.com) as a version control and the below commands will be for GIT.
  - NOTE: Windows users may want to install either of the software below:
    - 4265 Team uses [TortoiseGit](https://tortoisegit.org)
    - Another alternative is [Git For Windows](https://gitforwindows.org)

1. Open up a terminal session and navigate to a directory you want to clone the repository.
1. Use the following command in the terminal to clone this repository:
   ```bash
   git clone https://github.com/secret-city-wildbots/2024-Java-Testing.git
   ```
1. Now you should have a copy of the code locally you can test and learn with. You should have a directory called `2024-Java-Testing` and when going into that directory you should see some examples like: `00-*`, `01-*`, etc.

   ![Example of Directory When Cloned:](99-doc-images/Setup_GIT_Clone_Example.png)

## 00-java-hello-world

### Description

The goal of the example is to do the following:

1. Be able to connect and deploy to the RoboRIO
1. Be able to enable the robot
1. Print a statement repeatedly in the teleop periodic function

Note: This example is a `Timed Robot` code layout.

### Setup VSCode

To be able to use VSCode with this example, you will need to make sure you open VSCode to the specific folder, below are the steps:

  1. launch the VSCode application
  1. Go to `File` -> `New Window`
  1. While your new window is in focus, Go to `File` -> `Open Folder...`
  1. Select the directory `00-java-hello-world`
  1. Now you should be able to code and utilize all of the features built into the WPILib version of VSCode. Your VSCode should look like something below:
     ![Example VSCode View](99-doc-images/00-java-hello-world-vscode.png)

### Hardware Setup

Below is an image showing the minimum hardware needed to run and work with this example along with the wiring connections:

![Java Hello World Hardware Setup](99-doc-images/00-java_hello_world_setup.png)

### Code Overview

As mentioned in the description, we will be deploying the code to the RoboRIO, enabling the RoboRIO, switching to teleop mode, and observing the `Hello World!` statement being printed out in the console.

The main code lives in the `src/main/java/frc/robot` directory. There are 2 files:
  - Main.java
     - entry point for the program
  - Robot.java
    - This contains all of the init and periodic functions for the robot
       - robot
       - autonomous
       - teleop
       - disabled
       - test
       - simulation

you will notice on lines 79 - 83 in `Robot.java` the following code:

```java
/** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    System.out.println("Hello world!\n");
  }
```

The `periodic` function(s), will typically be called approx. every 20ms. Within that function we print out `Hello World!`.

### Running Example

1. After connecting everything as indicated in the diagram, power on the system
1. Connect to the Radio's wifi network
1. In VSCode, you will hit the `...` on the top right and click `Build Robot Code`. You should see `BUILD SUCCESSFUL` in the terminal of VSCode
   ![Build Code](99-doc-images/00-java_hello_world_build_code.png)
1. In VSCode, you will hit the `...` on the top right again and click `Deploy Robot Code`
1. Now open up the FRC Driver Station
1. Make sure `TeleOperated` is selected and then click the `Enable` button
   ![Driver Station](99-doc-images/00-java-frc-driver-station.png)
1. You should see the RSL (robot signal light) start to blink and the console start to fill up with `Hello World!`


