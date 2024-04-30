package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism;

public class StopMechanismCommand extends Command{
    private Mechanism mechSubsystem;

    public StopMechanismCommand(Mechanism mech) {
        mechSubsystem = mech;

        // This command requires the mech
        addRequirements(mechSubsystem);
    }

    @Override
    public void initialize() {
        // Stop the entire mech
        mechSubsystem.setBeltSpeed(0);
        mechSubsystem.setSourceSpeed(0);
        mechSubsystem.setAmpSpeed(0);
        mechSubsystem.setWheelState(false);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        // score in the amp?
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
