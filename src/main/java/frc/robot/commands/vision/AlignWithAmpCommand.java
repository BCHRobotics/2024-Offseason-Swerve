package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.Constants.VisionConstants.CameraMode;
import frc.robot.subsystems.Drivetrain;
import frc.utils.VisionUtils;
/*
 * Command that drives the robot to the amp using vision
 */

// WARNING - this is an untested class
public class AlignWithAmpCommand extends Command {
    // Since this command needs to command the robot,
    // it has a reference to the drivetrain
    public Drivetrain driveSubsystem;

    // These two variables represent the current position of the robot [FS],
    // and the goal position (position of target) [FS]
    private Pose2d startPosition;
    private Pose2d endPosition;

    // This represents how the robot should be offset from the target when finished [FS]
    private Transform2d endPositionOffset;
    
    public AlignWithAmpCommand(Drivetrain subsystem) {
        // Attach the drive subsystem to the command
        driveSubsystem = subsystem;
        
        // define the starting position, where the robot currently is
        startPosition = driveSubsystem.getPose();
        // define where the robot needs to go
        // TODO: think about reorganizing this so I can get rid of the public var and replace with a func
        endPosition = VisionUtils.getPose(CameraMode.AMP, driveSubsystem.isRedAlliance);
        // Define the desired offset from the target in TLS
        endPositionOffset = new Transform2d(CameraMode.AMP.getOffsets()[0], CameraMode.AMP.getOffsets()[1], new Rotation2d());
        // Transform the offset into FS
        endPositionOffset = VisionUtils.tagToField(endPositionOffset, VisionUtils.getPose(CameraMode.AMP, driveSubsystem.isRedAlliance).getRotation().getDegrees());
        
        // Require the drivetrain so that this command will cancel all other driving commands
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Set the drive mode to amp alignment
        driveSubsystem.setDriveMode(DriveModes.AMPALIGN);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // update the start position to the new robot position
        startPosition = driveSubsystem.getPose();
        // update the end position, in case the robot gets a better estimate of the tag position
        // TODO: keep a running average of target position so we're not just relying on the most recent frame
        endPosition = VisionUtils.getPose(CameraMode.AMP, driveSubsystem.isRedAlliance);

        // Create a variable that will represent the DIRECTION AND SPEED we need to go in
        Transform2d driveVector = new Transform2d(0, 0, new Rotation2d());
        // alignWithTagExact() will return null when there is an issue with the supplied variables, or the robot is done aligning
        // So, check to make sure neither is true before actually using the function
        if (VisionUtils.alignWithTagExact(endPosition, startPosition, endPositionOffset) != null) {
            // Calculate the DIRECTION AND SPEED we need to drive in
            driveVector = VisionUtils.alignWithTagExact(endPosition, startPosition, endPositionOffset);
        }

        // Tell the drivetrain to drive in the supplied direction
        driveSubsystem.drive(driveVector.getX(), driveVector.getY(), driveVector.getRotation().getDegrees(), true, true);
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        // TODO: score in the amp?
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // If the robot is done aligning alignWithTagExact() will be null, so finish the command
        // if the drive mode switches (driver cancels vision maybe) finish the command as well
        return VisionUtils.alignWithTagExact(endPosition, startPosition, endPositionOffset) == null || driveSubsystem.getDriveMode() != DriveModes.AMPALIGN;
    }
}