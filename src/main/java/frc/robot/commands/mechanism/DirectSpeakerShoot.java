package frc.robot.commands.mechanism;


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
 * Command that literally just shoots a note, regardless of elevator position and regardless of where the bot is
 */
public class DirectSpeakerShoot extends Command {
    Drivetrain driveSubsystem;
    
    public DirectSpeakerShoot(Drivetrain subsystem) {
        driveSubsystem = subsystem;
    }

    @Override
    public void initialize() {
        Mechanism.getInstance().spinWheels(12).schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            Mechanism.getInstance().scoreSpeaker(12).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        // End if the bot is lined up and ready to shoot, or if the driver cancels vision mode
        return Mechanism.getInstance().isCharged();
    }
}