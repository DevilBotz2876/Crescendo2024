This is the top-level folder for the DevilBotz robot code

The code uses Java WPILib.

# Getting Started
1. Install [WPILib](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html)
1. Install Git
    1. Windows
    1. MacOS
1. Get Code from GitHub
1. Simulating the Code
   1. Starting a Simulation
      * "F5" starts the simulation
         * Alternate Steps:
            * "Ctrl-Shift-P" or "Command-Shift-P"
            * "Simulate Robot Code"
      * Check Only: "Sim GUI"
         * If you get the following error: "cannot find frc.robot.main", then:
            * "Ctrl-Shift-P" --> "Clean Language Server Workspace"
   1. Robot Simulation (aka Sim GUI) Overview
      1. Configure XBox Controller
         1. Make sure joystick is in "X-Box Mode". Plug in Joystick.
         1. Verify Joystick is Detected in "System Joysticks" window
            1. On Mac OS: You must connect via a bluetooth wireless enabled controller
         1. Drag controller from "Systeddm Joysticks" to "Joysticks" (Joystick[0])
         1. Make sure to "Map Gamepad" is checked
      1. Open 2D Field View
         1. NetworkTables --> "SmartDashboard" --> "Field"
         1. To Hide Swerve Modules:
            1. Click on the "Hamburger Button"
            1. Goto "XModules"
               * Box/Image --> Hidden
               * Arrow Size --> 10% (slider)
      1. Set Robot State
         1. Set Mode to "Teleoperated"
         1. Thumb Controls should now move the robot
   1. 2D Mechanism Simulation
      1. NetworkTables --> "SmartDashboard" --> "Inferno 2D Simulation"
         ![Inferno2DSimulation](https://github.com/DevilBotz2876/Crescendo2024/assets/115738405/563381bc-6ee0-4b3d-a01f-8f6d709c51e1)
         * Shooter (Blue/Grey)
         * Intake (Red/Grey)
         * Note (Orange)
         * Arm (Yellow/Orange)
         * Climber (Grey)
         * LED (Bottom right box)
   1. Vision Field
       1.  NetworkTables --> "SmartDashboard" --> "VisionSystemSim-main" --> Sim Field
   1. Alliance Selection
   1. Autonomous
   1. AdvantageScope

1. Contributing to the Repo
    1. [Optional] Create/Login to GitHub
    1. Request write access to the repo
    1. Configure git inside VS Code
       * Username
       * Email

* Separate document on contibuting to Repo


# Useful Resources:
* [FRC Programming in WPILib](https://youtube.com/playlist?list=PL4GNHenJg9JD5xdRxByaZZEZP1PPajPeV&si=1dgx1ZFQP8GEq4w_)
* [Git Tutorial](https://learngitbranching.js.org/)
* [Java Tutorial](https://youtube.com/playlist?list=PLZPZq0r_RZOMhCAyywfnYLlrjiVOkdAI1&si=ImrjG_c4-wThJqy3)
* [WPILib Documentation](https://docs.wpilib.org/en/stable/)

# Directory Structure
* [src/main](src/main): All of the robot code and config files are here
* [vendordeps](vendordeps): Contains the various external libraries and versions that the code base utilizes.
* [.github/workflows](.github/workflows): Contains various GitHub automation scripts that are run automatically at various stages of the code development/integration process.
