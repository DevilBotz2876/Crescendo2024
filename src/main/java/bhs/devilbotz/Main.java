// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Main initialization function. Do not perform any initialization here.
 */
public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
