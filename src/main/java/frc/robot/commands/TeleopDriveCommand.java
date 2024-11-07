package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.subsystems.Drivetrain;

/*
 * Command that drives the robot based on joystick input
 * Used for normal teleop driving (no vision)
 */
public class TeleopDriveCommand extends Command {
    
    private Drivetrain driveSubsystem;

    DoubleSupplier commandX;
    DoubleSupplier commandY;
    DoubleSupplier commandRot;
    BooleanSupplier isFieldRelative;
    BooleanSupplier isRateLimited;

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
        System.out.println("MANUAL DRIVING ENGAGED");
    }

    @Override
    public void execute() {
        driveSubsystem.drive(commandX.getAsDouble(), commandY.getAsDouble(), commandRot.getAsDouble(), isFieldRelative.getAsBoolean(), isRateLimited.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("INTERRUPT!");
        }
        else {
            System.out.println("MANUAL DRIVING OFF");
        }
    }

    @Override
    public boolean isFinished() {
        // End if the drive mode is not manual
        return driveSubsystem.getDriveMode() != DriveModes.MANUAL;
    }
}
