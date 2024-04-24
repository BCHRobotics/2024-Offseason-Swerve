package frc.robot.commands.vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.Drivetrain;

// UNFINISHED
public class AlignWithNoteCommand extends Command {
    public Drivetrain driveSubsystem;

    private double rotationSpeed;

    DoubleSupplier commandX;
    DoubleSupplier commandY;
    DoubleSupplier commandRot;
    BooleanSupplier isFieldRelative;
    BooleanSupplier isRateLimited;
    
    public AlignWithNoteCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed, BooleanSupplier fieldRelative, BooleanSupplier rateLimit, Drivetrain subsystem) {
        driveSubsystem = subsystem;

        commandX = xSpeed;
        commandY = ySpeed;
        commandRot = rotSpeed;
        isFieldRelative = fieldRelative;
        isRateLimited = rateLimit;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Set the drive mode
        driveSubsystem.setDriveMode(DriveModes.NOTEALIGN);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveSubsystem.drive(commandX.getAsDouble(), commandY.getAsDouble(), commandRot.getAsDouble(), isFieldRelative.getAsBoolean(), isRateLimited.getAsBoolean());
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        new TeleopDriveCommand(commandY, commandX, commandRot, isFieldRelative, isRateLimited, driveSubsystem).schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return driveSubsystem.getDriveMode() != DriveModes.SPEAKERALIGN;
    }
}
