package frc.robot.commands.vision;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism;

/*
 * Command that charges the wheels, then shoots when the robot is lined up
 * Currently being used to test trajectory prediction
 * Does not use a drive mode because the command doesn't drive the robot
 */
public class SpeakerTargetingCommand extends Command{
    Mechanism mechSubsystem;
    BooleanSupplier shotReady;

    public SpeakerTargetingCommand(Mechanism mech, BooleanSupplier canShoot) {
        mechSubsystem = mech;
        shotReady = canShoot;
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
        return (mechSubsystem.isCharged() && shotReady.getAsBoolean());
    }
}
