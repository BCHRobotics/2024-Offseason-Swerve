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
    // This command needs to command the drivetrain, so we have a references here
    private Drivetrain driveSubsystem;

    // These three are inputs for x, y, and rot speed from the driver
    // NOTE - these are NOT DOUBLES, they are DOUBLE SUPPLIERS, which point to a double value
    // this is the easiest way to get the updated speeds, instead of assigning the variables over and over
    DoubleSupplier commandX;
    DoubleSupplier commandY;
    DoubleSupplier commandRot;

    // These two are settings, whether to use ratelimiting and whether to interpret the commands as FS (true) or LS (false)
    BooleanSupplier isFieldRelative;
    BooleanSupplier isRateLimited;

    public TeleopDriveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed, BooleanSupplier fieldRelative, BooleanSupplier rateLimit, Drivetrain subsystem) {
        // Assign the variables that point to input values
        commandX = xSpeed;
        commandY = ySpeed;
        commandRot = rotSpeed;
        
        isFieldRelative = fieldRelative;
        isRateLimited = rateLimit;

        driveSubsystem = subsystem;
        
        // This command requires the drivetrain so that it cannot run at the same time as other driving commands
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Set the drive mode
        driveSubsystem.setDriveMode(DriveModes.MANUAL);
        // Tell the driver that manual driving has been enabled
        System.out.println("MANUAL DRIVING ENGAGED");
    }

    @Override
    public void execute() {
        driveSubsystem.drive(commandX.getAsDouble(), commandY.getAsDouble(), commandRot.getAsDouble(), isFieldRelative.getAsBoolean(), isRateLimited.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        // Check to see if the command was canceled by another command or if it ended itself
        if (interrupted) {
            // This will happen most of the time, e.g. when switching to vision
            System.out.println("INTERRUPT!");
        }
        else {
            // This happens when the driving mode switches off manual
            // Doesn't usually happen, for example when a vision command is triggered 
            // it is setup before the drive mode is set
            // So the program interrupts this command (because of the new one)
            // before it realizes the driveMode isn't manual anymore
            System.out.println("MANUAL DRIVING OFF");
        }
    }

    @Override
    public boolean isFinished() {
        // End if the drive mode is not manual
        return driveSubsystem.getDriveMode() != DriveModes.MANUAL;
    }
}
