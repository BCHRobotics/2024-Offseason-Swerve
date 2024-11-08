// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.commands.CombinedCommands;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.vision.AlignWithAmpCommand;
import frc.robot.commands.mechanism.DirectSpeakerShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mechanism;
import frc.utils.devices.BeamBreak;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final Drivetrain m_robotDrive = new Drivetrain();
    private final Elevator m_elevator;
    private final Mechanism m_mechanism;

    // Flightstick controller
    //CommandJoystick m_driverFlightstickController = new CommandJoystick(OIConstants.kFlightstickPort);
    // Driving controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDrivingControllerXBoxPort);
    // Operator controller
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatingControllerXBoxPort);
    // A set of commands that do multiple things at once (raise elevator, then shoot)
    CombinedCommands m_combinedCommands = new CombinedCommands();

    // The auto chooser
    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_elevator = Elevator.getInstance();
        m_mechanism = Mechanism.getInstance();

        configureNamedCommands();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // Sets up the drivetrain for teleoperated driving
    public void configureDriveMode(boolean isRedAlliance) {
        final double invert = isRedAlliance ? -1 : 1;
        
        // If no other command is running on the drivetrain, then this manual driving command (driving via controller) is used
        m_robotDrive.setDefaultCommand(new TeleopDriveCommand(
            () -> -m_driverController.getLeftY() * invert,
            () -> -m_driverController.getLeftX() * invert,
            () -> -m_driverController.getRightX(),
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive));
        
        // Setup the commands associated with all buttons on the controller
        // Driver controller
        configureButtonBindingsDriver(isRedAlliance);
        // Operator controller
        configureButtonBindingsOperator(isRedAlliance);

        // Set the alliance to either red or blue (to invert controls if necessary)
        m_robotDrive.setAlliance(isRedAlliance);
    }

    /**
     * Method for configuring named commands 
     * (used during autos)
     */
    public void configureNamedCommands() {
        // Enabling and disabling note vision in auto
        NamedCommands.registerCommand("ALIGN NOTE", new InstantCommand(() -> m_robotDrive.autoVision(true)));
        NamedCommands.registerCommand("CANCEL ALIGN", new InstantCommand(() -> m_robotDrive.autoVision(false)));

        NamedCommands.registerCommand("INTAKE", m_mechanism.groundIntakeAuto(12));
        NamedCommands.registerCommand("RELEASE", m_mechanism.groundReleaseAuto(12));
        NamedCommands.registerCommand("AMP SCORE", m_mechanism.scoreAmp(6));
        NamedCommands.registerCommand("ELEVATOR LOW", m_elevator.moveToPositionCommand(ElevatorPositions.INTAKE));
        NamedCommands.registerCommand("ELEVATOR HIGH", m_elevator.moveToPositionCommand(ElevatorPositions.AMP));
        NamedCommands.registerCommand("SPEAKER SCORE", new RunCommand(() -> this.m_mechanism.spinWheels(12)).until(() -> this.m_mechanism.isCharged()).andThen(this.m_mechanism.scoreSpeaker(12)));
        NamedCommands.registerCommand("SPIN", this.m_mechanism.spinWheels(12));
    }

    /**
     * Binding for driver xbox controller buttons
     * 
     * NOTE - Things are configured for the SHSM event, vision is (ofc) not being used for this
     */
    private void configureButtonBindingsDriver(boolean isRedAlliance) {
        // variable is not needed rn
        //final double invert = isRedAlliance ? -1 : 1;

        // NOTE FOR SLOW/FAST MODE COMMANDS
        // These commands don't have requirements else they interrupt the drive command (TeleopDriveCommand)

        // Slow mode command (Left Bumper)
        this.m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));
        this.m_driverController.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

        // Fast mode command (Right Bumper)
        this.m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));
        this.m_driverController.rightBumper().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));
        
        // Activate targeting  (not used rn)
        this.m_driverController.rightTrigger().onTrue(
            new AlignWithAmpCommand(m_robotDrive));

        // Shoot directly at speaker (not used rn)
        // this.m_driverController.leftTrigger().whileTrue(
        //     new AlignWithSpeakerCommand(m_robotDrive));
        this.m_driverController.leftTrigger().onTrue(
            new DirectSpeakerShoot(m_robotDrive));

        // Ground intake
        this.m_driverController.x().onTrue(this.m_mechanism.groundIntake(12));
        // Stop the mech
        this.m_driverController.b().onTrue(this.m_mechanism.stopMechanism());
        // Scoring into amp
        this.m_driverController.a().onTrue(m_mechanism.scoreAmp(6));
        // Intaking from source
        this.m_driverController.povLeft().onTrue(this.m_combinedCommands.pickupFromSource());

        // Reset Gyro
        this.m_driverController.y().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
        // // Set drive mode to manual (cancel vision) (not used rn)
        // this.m_driverController.a().onTrue(new InstantCommand(() -> m_robotDrive.setDriveMode(DriveModes.MANUAL)));

        // Moving the elevator (TO BE TESTED)
        this.m_driverController.povUp().onTrue(this.m_elevator.moveToPositionCommand(ElevatorPositions.AMP));
        this.m_driverController.povRight().onTrue(this.m_elevator.moveToPositionCommand(ElevatorPositions.SOURCE));
        this.m_driverController.povDown().onTrue(this.m_elevator.moveToPositionCommand(ElevatorPositions.INTAKE));

        // Lock heading to amp direction (not used rn)
        // this.m_driverController.povLeft().onTrue(
        //     new HeadingLockDriveCommand(
        //     () -> -m_driverController.getLeftY() * invert,
        //     () -> -m_driverController.getLeftX() * invert,
        //     () -> -m_driverController.getRightX(),
        //     () -> OIConstants.kFieldRelative, 
        //     () -> OIConstants.kRateLimited,
        //     m_robotDrive));
    }

    /**
     * Binding for operator xbox controller buttons
     */
    private void configureButtonBindingsOperator(boolean isRedAlliance) {
        // -- no operator controls because working with 2 controllers is annoying -- //

        // Request intake (ground and source)
        // this.m_operatorController.leftBumper().onTrue(new InstantCommand(() -> m_mechanism.requestIntake(1)));
        // this.m_operatorController.rightBumper().onTrue(new InstantCommand(() -> m_mechanism.requestIntake(2)));
       
        // // Cancel command
        // this.m_operatorController.a().onTrue(this.m_mechanism.stopMechanism());
        // // Intaking from source
        // this.m_operatorController.y().onTrue(this.m_combinedCommands.pickupFromSource());
        // // Intaking from ground
        // this.m_operatorController.x().onTrue(this.m_mechanism.groundIntake(12));
        // // Scoring into amp
        // this.m_operatorController.b().onTrue(m_mechanism.scoreAmp(6));

        // // Speaker score
        // this.m_operatorController.povLeft().onTrue(new RunCommand(() -> this.m_mechanism.spinWheels(12)).until(() -> this.m_mechanism.isCharged()).andThen(this.m_mechanism.scoreSpeaker(12)));
        // // Release note onto floor
        // this.m_operatorController.leftTrigger().onTrue(this.m_mechanism.groundReleaseAuto(12));

        // // Spin up the speaker wheels
        // this.m_operatorController.rightTrigger().onTrue(this.m_mechanism.spinWheels(12));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */ 
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * This function is called when the robot enters disabled mode, it sets the motors to brake mode.
     */
    public void eStop() {
        m_robotDrive.setIdleStates(1);
    }

    /**
     * enable the PCM channels
     */
    public void enablePCMChannels() {
        BeamBreak.solenoidChannelActive(true);
        
    }

    /**
     * Initializes the LEDs
     */
    public void initLEDs() {
        this.m_mechanism.powerLEDs(LEDColor.OFF);
    }
}
