package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.Constants.VisionConstants.CameraMode;
import frc.robot.subsystems.Drivetrain;
import frc.utils.VisionUtils;

// UNTESTED
public class AlignWithAmpCommand extends Command {
    public Drivetrain driveSubsystem;

    private Pose2d startPosition;
    private Pose2d endPosition;

    private Transform2d endPositionOffset;
    
    public AlignWithAmpCommand(Drivetrain subsystem) {
        driveSubsystem = subsystem;
        
        // get the data needed to align with the amp
        startPosition = driveSubsystem.getPose();
        endPosition = driveSubsystem.ampTargetPose;
        endPositionOffset = new Transform2d(CameraMode.AMP.getOffsets()[0], CameraMode.AMP.getOffsets()[1], new Rotation2d());

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Set the drive mode
        driveSubsystem.setDriveMode(DriveModes.AMPALIGN);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        startPosition = driveSubsystem.getPose();
        endPosition = driveSubsystem.ampTargetPose;

        Transform2d driveVector = new Transform2d(0, 0, new Rotation2d());
        if (VisionUtils.alignWithTagExact(endPosition, startPosition, endPositionOffset) != null) {
            driveVector = VisionUtils.alignWithTagExact(endPosition, startPosition, endPositionOffset);
        }

        driveSubsystem.drive(driveVector.getX(), driveVector.getY(), driveVector.getRotation().getDegrees(), true, true);
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        // score in the amp?
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return VisionUtils.alignWithTagExact(endPosition, startPosition, endPositionOffset) == null || driveSubsystem.getDriveMode() != DriveModes.AMPALIGN;
    }
}