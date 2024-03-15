// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.config.RobotConfig.ArmConstants;
import frc.robot.subsystems.arm.Arm;

public class ArmToPositionPID extends PIDCommand {
  
  /** Creates a new ArmToPositionTP. */
  public ArmToPositionPID(DoubleSupplier positionDegrees, Arm arm) {

    super(
        new PIDController(ArmConstants.pidKp, ArmConstants.pidKi, ArmConstants.pidKd),
    () -> arm.getAngle(),
    positionDegrees,
    output -> {
      ArmFeedforward armFeedforward = new ArmFeedforward(0, ArmConstants.ffKg, ArmConstants.ffKv, ArmConstants.ffKa);
      double ff = armFeedforward.calculate(positionDegrees.getAsDouble(), 0);
      arm.runVoltage(output+ff);
      Logger.recordOutput("Arm/PID/ff", ff);
      Logger.recordOutput("Arm/PID/output", output);
    },
    arm
    );
    
    //getController().setTolerance(5,10);
    
  }

  public boolean isFinished() {
    boolean done = super.isFinished();
    Logger.recordOutput("Arm/PID/isFinished", done);
    return done;
  }
}
