package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Mechanism;

/*
 * Command that charges the wheels, then shoots when the robot is lined up
 * Currently being used to test trajectory prediction
 * Does not use a drive mode because the command doesn't drive the robot
 */
public class SpeakerTargetingCommand extends Command{
    Mechanism mechSubsystem;
    Drivetrain driveSubsystem;

    public SpeakerTargetingCommand(Mechanism mech, Drivetrain drive) {
        mechSubsystem = mech;
        driveSubsystem = drive;

        addRequirements(driveSubsystem);
        addRequirements(mechSubsystem);
    }

    @Override
    public void initialize() {
        // Set the drive mode
        System.out.println("TARGETING ENGAGED");
        // Charge the wheels
        mechSubsystem.spinWheels(12).schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        // When ended, the command will print to the console and shoot a note
        System.out.println("TAKING SHOT...");
        mechSubsystem.scoreSpeaker(12).schedule();
    }

    @Override
    public boolean isFinished() {
        // End the command if the wheels are charged and the shot will hit the speaker
        return (mechSubsystem.isCharged() && driveSubsystem.onTarget);
    }
}
