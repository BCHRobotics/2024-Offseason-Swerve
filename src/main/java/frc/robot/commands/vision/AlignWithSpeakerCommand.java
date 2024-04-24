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

// UNTESTED
public class AlignWithSpeakerCommand extends Command {
    public Drivetrain driveSubsystem;

    private Pose2d startPosition;
    private Pose2d endPosition;

    private Transform2d endPositionOffset;
    
    public AlignWithSpeakerCommand(Drivetrain subsystem) {
        driveSubsystem = subsystem;

        // get the data needed to align with the speaker
        startPosition = driveSubsystem.getPose();
        endPosition = driveSubsystem.speakerTargetPose;
        endPositionOffset = new Transform2d(CameraMode.SPEAKER.getOffsets()[0], CameraMode.SPEAKER.getOffsets()[1], new Rotation2d());

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Set the drive mode
        System.out.println("VISION START");
        driveSubsystem.setDriveMode(DriveModes.SPEAKERALIGN);

        Mechanism.getInstance().spinWheels(12).schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        startPosition = driveSubsystem.getPose();
        endPosition = driveSubsystem.speakerTargetPose;

        Transform2d driveVector = new Transform2d(0, 0, new Rotation2d());
        if (VisionUtils.alignWithTagRadial(endPosition, startPosition, endPositionOffset.getX()) != null) {
            driveVector = VisionUtils.alignWithTagRadial(endPosition, startPosition, endPositionOffset.getX());
        }

        driveSubsystem.drive(driveVector.getX(), driveVector.getY(), driveVector.getRotation().getDegrees(), true, true);
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        System.out.println("VISION END");
        Mechanism.getInstance().scoreSpeaker(12).schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return VisionUtils.alignWithTagRadial(endPosition, startPosition, endPositionOffset.getX()) == null || driveSubsystem.getDriveMode() != DriveModes.SPEAKERALIGN;
    }
}
