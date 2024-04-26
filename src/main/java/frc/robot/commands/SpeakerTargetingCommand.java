package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Mechanism;

public class SpeakerTargetingCommand extends Command{
    Mechanism mechSubsystem;
    Drivetrain driveSubsystem;

    // pass in the subsystems to the command
    public SpeakerTargetingCommand(Mechanism mech, Drivetrain drive) {
        mechSubsystem = mech;
        driveSubsystem = drive;
    }

    @Override
    public void initialize() {
        // Set the drive mode
        System.out.println("TARGETING ENGAGED");
        mechSubsystem.spinWheels(12).schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        System.out.println("TAKING SHOT...");
        mechSubsystem.scoreSpeaker(12).schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (mechSubsystem.isCharged() && driveSubsystem.onTarget);
    }
}
