The config directory is used to support multiple robot versions using the same code base.

# 2023
We tried to support multiple robots using the same code base by:
   * Creating a JSON config file specific to each robot
   * Loading the JSON config file by detecting the RoboRio MAC Address

Shortcomings:
   * No way to share common/default settings. If a new setting was added, it needed to be added to all config files, otherwise a runtime error could occur.
   * JSON does not support comments, so difficult to document important information for certain configurations
   * Loading/parsing the JSON file was slow
   * When coding, difficult to determine which/where the config setting source
   * Difficult to simulate robot functionality

# 2024
We had 3 different robots with different capabilities for dev/testing purposes:

1. Sherman: Tank Drive + Initial Arm Prototype
2. Phoenix: Initial Swerve Drive + Vision Prototype
3. Inferno: Final Robot

A "RobotConfig" class
