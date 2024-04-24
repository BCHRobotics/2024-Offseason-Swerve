package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.subsystems.Drivetrain;

// Command for driving the robot
// UNTESTED
public class TeleopDriveCommand extends Command {
    
    private Drivetrain driveSubsystem;

    DoubleSupplier commandX;
    DoubleSupplier commandY;
    DoubleSupplier commandRot;
    BooleanSupplier isFieldRelative;
    BooleanSupplier isRateLimited;

    // Constructor for the command
    public TeleopDriveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed, BooleanSupplier fieldRelative, BooleanSupplier rateLimit, Drivetrain subsystem) {
        commandX = xSpeed;
        commandY = ySpeed;
        commandRot = rotSpeed;
        isFieldRelative = fieldRelative;
        isRateLimited = rateLimit;

        driveSubsystem = subsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Set the drive mode
        driveSubsystem.setDriveMode(DriveModes.MANUAL);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveSubsystem.drive(commandX.getAsDouble(), commandY.getAsDouble(), commandRot.getAsDouble(), isFieldRelative.getAsBoolean(), isRateLimited.getAsBoolean());
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        // nothing here yet
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return driveSubsystem.getDriveMode() != DriveModes.MANUAL;
    }
}
