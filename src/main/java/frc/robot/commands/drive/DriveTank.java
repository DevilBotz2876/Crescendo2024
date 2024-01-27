package frc.robot.commands.drive;

import frc.robot.subsystems.drive.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class DriveTank extends Command {
    private final DriveTrain drive;
    private final DoubleSupplier speed;
    private final DoubleSupplier rotation;

    public DriveTank(DriveTrain drive, DoubleSupplier speed, DoubleSupplier rotation) {
        this.drive = drive;
        this.speed = speed;
        this.rotation = rotation;
        addRequirements(this.drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive.arcadeDrive(
          speed.getAsDouble(),
          rotation.getAsDouble()
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}