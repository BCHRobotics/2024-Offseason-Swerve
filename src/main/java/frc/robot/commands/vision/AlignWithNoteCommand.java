package frc.robot.commands.vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.Drivetrain;

// WARNING - unfinished command
public class AlignWithNoteCommand extends Command {
    public Drivetrain driveSubsystem;

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

    @Override
    public void execute() {
        // as of now this just drives the bot normally, will incorporate camera into this later
        driveSubsystem.drive(commandX.getAsDouble(), commandY.getAsDouble(), commandRot.getAsDouble(), isFieldRelative.getAsBoolean(), isRateLimited.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        new TeleopDriveCommand(commandY, commandX, commandRot, isFieldRelative, isRateLimited, driveSubsystem).schedule();
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.getDriveMode() != DriveModes.SPEAKERALIGN;
    }
}
