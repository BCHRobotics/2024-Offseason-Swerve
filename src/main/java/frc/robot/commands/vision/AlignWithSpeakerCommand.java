package frc.robot.commands.vision;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.Constants.VisionConstants.CameraMode;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Mechanism;
import frc.utils.VisionUtils;

/*
 * Command that points towards the speaker and lines up with it, then shoots
 * Uses the radial vision profile to accomplish this
 */
// WARNING - unfinished command
public class AlignWithSpeakerCommand extends Command {
    public Drivetrain driveSubsystem;

    private Pose2d startPosition;
    private Pose2d endPosition;

    private Transform2d endPositionOffset;
    
    public AlignWithSpeakerCommand(Drivetrain subsystem) {
        driveSubsystem = subsystem;

        // get the data needed to align with the speaker
        startPosition = driveSubsystem.getPose();
        endPosition = VisionUtils.getPose(CameraMode.SPEAKER, driveSubsystem.isRedAlliance);
        endPositionOffset = new Transform2d(CameraMode.SPEAKER.getOffsets()[0], CameraMode.SPEAKER.getOffsets()[1], new Rotation2d());

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Set the drive mode to speaker alignment
        driveSubsystem.setDriveMode(DriveModes.SPEAKERALIGN);

        Mechanism.getInstance().spinWheels(12).schedule();
    }

    @Override
    public void execute() {
        startPosition = driveSubsystem.getPose();
        endPosition = VisionUtils.getPose(CameraMode.SPEAKER, driveSubsystem.isRedAlliance);

        Transform2d driveVector = new Transform2d(0, 0, new Rotation2d());
        if (VisionUtils.alignWithTagRadial(endPosition, startPosition, endPositionOffset.getX()) != null) {
            driveVector = VisionUtils.alignWithTagRadial(endPosition, startPosition, endPositionOffset.getX());
        }

        driveSubsystem.drive(driveVector.getX(), driveVector.getY(), driveVector.getRotation().getDegrees(), true, true);
    }

    @Override
    public void end(boolean interrupted) {
        if (driveSubsystem.getDriveMode() == DriveModes.SPEAKERALIGN) {
            Mechanism.getInstance().scoreSpeaker(12).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        // End if the bot is lined up and ready to shoot, or if the driver cancels vision mode
        return (VisionUtils.alignWithTagRadial(endPosition, startPosition, endPositionOffset.getX()) == null && Mechanism.getInstance().isCharged()) || driveSubsystem.getDriveMode() != DriveModes.SPEAKERALIGN;
    }
}
