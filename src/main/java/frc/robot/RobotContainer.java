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
import frc.robot.commands.HeadingLockDriveCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.vision.AlignWithAmpCommand;
import frc.robot.commands.vision.AlignWithSpeakerCommand;
import frc.robot.Constants.DriveConstants.DriveModes;
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
    private final CombinedCommands m_combinedCommands = new CombinedCommands();

    // Flightstick controller
    //CommandJoystick m_driverFlightstickController = new CommandJoystick(OIConstants.kFlightstickPort);
    // Driving controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDrivingControllerXBoxPort);
    // Operator controller
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatingControllerXBoxPort);

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

    public void configureDriveMode(boolean isRedAlliance) {
        final double invert = isRedAlliance ? -1 : 1;
        
        m_robotDrive.setDefaultCommand(new TeleopDriveCommand(
            () -> -m_driverController.getLeftY() * invert,
            () -> -m_driverController.getLeftX() * invert,
            () -> -m_driverController.getRightX(),
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive));

        /* 
         * CURRENT BUTTON LAYOUT (subject to change):
         * 
         * -- Driver Controller -- 
         * 
         * Y -- RESET GYRO
         * LEFT TRIGGER -- BRAKE
         * LEFT BUMPER -- TOGGLE SLOW MODE
         * RIGHT TRIGGER -- ALIGN WITH SPEAKER
         * X -- ALIGN WITH AMP
         * B -- ALIGN WITH NOTE
         * A -- CANCEL ALIGN
         * POV UP, DOWN, LEFT, RIGHT -- HEADING LOCKS
         * 
         * -- Operator Controller --
         * 
         * POV UP -- ELEVATOR TO AMP
         * POV RIGHT -- ELEVATOR TO SOURCE
         * POV DOWN -- ELEVATOR TO GROUND
         * LEFT BUMPER -- REQUEST GROUND INTAKE
         * RIGHT BUMPER -- REQUEST SOURCE INTAKE
         * POV LEFT -- SCORE SPEAKER (CHARGE WHEELS IF NOT CHARGED)
         * B -- SCORE AMP 
         * Y -- SOURCE INTAKE
         * X -- GROUND INTAKE
         * A -- CANCEL INTAKE
         * LEFT TRIGGER -- RELEASE NOTE
         * RIGHT TRIGGER -- CHARGE SPEAKER WHEELS
         */

        configureButtonBindingsDriver(isRedAlliance);
        configureButtonBindingsOperator(isRedAlliance);

        m_robotDrive.setAlliance(isRedAlliance);
    }

    /**
     * Method for configuring named commands 
     * (used during autos)
     */
    public void configureNamedCommands() {
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
     */
    private void configureButtonBindingsDriver(boolean isRedAlliance) {
        final double invert = isRedAlliance ? -1 : 1;

        // Brake command (Left Trigger)
        this.m_driverController.leftTrigger().whileTrue(new RunCommand(() -> m_robotDrive.setX(),m_robotDrive));
        // Slow mode command (Left Bumper)
        this.m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true), m_robotDrive));
        this.m_driverController.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false), m_robotDrive));

        // Slow mode command (Right Bumper)
        this.m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true), m_robotDrive));
        this.m_driverController.rightBumper().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false), m_robotDrive));
        
        // Align with speaker
        this.m_driverController.rightTrigger().onTrue(
            new AlignWithSpeakerCommand(m_robotDrive));

        // Align with amp
        this.m_driverController.x().onTrue(
            new AlignWithAmpCommand(m_robotDrive));

        // Reset Gyro
        this.m_driverController.y().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
        // Cancel Alignment
        this.m_driverController.a().onTrue(new InstantCommand(() -> m_robotDrive.setDriveMode(DriveModes.MANUAL)));

        // Lock heading to amp direction
        this.m_driverController.povLeft().onTrue(
            new HeadingLockDriveCommand(
            () -> -m_driverController.getLeftY() * invert,
            () -> -m_driverController.getLeftX() * invert,
            () -> -m_driverController.getRightX(),
            () -> OIConstants.kFieldRelative, 
            () -> OIConstants.kRateLimited,
            m_robotDrive));
    }

    /**
     * Binding for operator xbox controller buttons
     */
    private void configureButtonBindingsOperator(boolean isRedAlliance) {
        // Moving the elevator
        this.m_operatorController.povUp().onTrue(this.m_elevator.moveToPositionCommand(ElevatorPositions.AMP));
        this.m_operatorController.povRight().onTrue(this.m_elevator.moveToPositionCommand(ElevatorPositions.SOURCE));
        this.m_operatorController.povDown().onTrue(this.m_elevator.moveToPositionCommand(ElevatorPositions.INTAKE));
        // Request intake (ground and source)
        this.m_operatorController.leftBumper().onTrue(new InstantCommand(() -> m_mechanism.requestIntake(1)));
        this.m_operatorController.rightBumper().onTrue(new InstantCommand(() -> m_mechanism.requestIntake(2)));
       
        // Cancel command
        this.m_operatorController.a().onTrue(this.m_mechanism.stopMechanism());
        // Intaking from source
        this.m_operatorController.y().onTrue(this.m_combinedCommands.pickupFromSource());
        // Intaking from ground
        this.m_operatorController.x().onTrue(this.m_mechanism.groundIntake(12));
        // Scoring into amp
        this.m_operatorController.b().onTrue(m_mechanism.scoreAmp(6));

        // Speaker score
        this.m_operatorController.povLeft().onTrue(new RunCommand(() -> this.m_mechanism.spinWheels(12)).until(() -> this.m_mechanism.isCharged()).andThen(this.m_mechanism.scoreSpeaker(12)));
        // Release note onto floor
        this.m_operatorController.leftTrigger().onTrue(this.m_mechanism.groundReleaseAuto(12));

        // Spin up the speaker wheels
        this.m_operatorController.rightTrigger().onTrue(this.m_mechanism.spinWheels(12));
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
