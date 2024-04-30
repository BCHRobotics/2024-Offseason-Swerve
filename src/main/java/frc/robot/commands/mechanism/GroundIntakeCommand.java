package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.subsystems.Mechanism;
import frc.utils.VisionUtils;

public class GroundIntakeCommand extends Command {
    private Mechanism mechSubsystem; 

    public GroundIntakeCommand(Mechanism mech) {
        mechSubsystem = mech;
    }
}
