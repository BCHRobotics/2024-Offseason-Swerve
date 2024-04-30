// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.utils.devices.BeamBreak;
import frc.utils.devices.LEDs;
import frc.utils.devices.BeamBreak.Phase;

public class Mechanism extends SubsystemBase{
    private static Mechanism instance = null;
    
    // The beam-break sensor that detects where a note is in the mechanism
    private final BeamBreak m_beamBreak = new BeamBreak();
    private LEDs m_LEDs = new LEDs();
    private Elevator m_elevator;

    // The phase of the beam-break sensor
    private Phase m_currentPhase = Phase.NONE;

    private int requestIntakeType;
    private boolean areWheelsCharged;

    private final CANSparkMax m_bottomBeltMotor = new CANSparkMax(MechanismConstants.kBottomBeltMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_topBeltMotor = new CANSparkMax(MechanismConstants.kTopBeltMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_sourceMotor = new CANSparkMax(MechanismConstants.kSourceMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_ampMotor = new CANSparkMax(MechanismConstants.kAmpMotorCanId, MotorType.kBrushless);

    /** Creates a new Mechanism. */
    public Mechanism() {
        m_elevator = Elevator.getInstance();

        this.m_bottomBeltMotor.restoreFactoryDefaults();
        this.m_topBeltMotor.restoreFactoryDefaults();
        this.m_sourceMotor.restoreFactoryDefaults();
        this.m_ampMotor.restoreFactoryDefaults();

        this.m_bottomBeltMotor.setIdleMode(IdleMode.kBrake);
        this.m_topBeltMotor.setIdleMode(IdleMode.kBrake);
        this.m_sourceMotor.setIdleMode(IdleMode.kBrake);
        this.m_ampMotor.setIdleMode(IdleMode.kBrake);

        this.m_bottomBeltMotor.setSmartCurrentLimit(40, 20); //keep 40 for neo550
        this.m_topBeltMotor.setSmartCurrentLimit(40, 20); //keep 40 for neo550
        this.m_sourceMotor.setSmartCurrentLimit(60, 20);
        this.m_ampMotor.setSmartCurrentLimit(60, 20);

        this.m_bottomBeltMotor.setInverted(true);
        this.m_topBeltMotor.setInverted(true);
        this.m_sourceMotor.setInverted(false);
        this.m_ampMotor.setInverted(false);

        this.m_bottomBeltMotor.setOpenLoopRampRate(0.05);
        this.m_topBeltMotor.setOpenLoopRampRate(0.05);
        this.m_sourceMotor.setOpenLoopRampRate(0.05);
        this.m_ampMotor.setOpenLoopRampRate(0.05);

        this.m_bottomBeltMotor.enableVoltageCompensation(12);
        this.m_topBeltMotor.enableVoltageCompensation(12);
        this.m_sourceMotor.enableVoltageCompensation(12);
        this.m_ampMotor.enableVoltageCompensation(12);

        requestIntakeType = 0;
    }

    /**
     * Gets the instance of the mechanism
     * @return the instance of the mechanism
     */
    public static Mechanism getInstance() {
        if (instance == null) {
            instance = new Mechanism();
        }
        return instance;
    }

    /**
     * updates the phase of the beam break sensor
     */
    public void updatePhase() {
        this.m_beamBreak.updatePhase();
        this.m_currentPhase = this.m_beamBreak.getPhase();
    }

    /**
     * Sets the speed of the belt motor
     * @param speed the speed in volts [0 --> 12]
     */
    public void setBeltSpeed(double speed) {
        this.m_bottomBeltMotor.setVoltage(speed);
        this.m_topBeltMotor.setVoltage(speed);
    }

    /**
     * Sets the speed of the source motor
     * @param speed the speed in volts [0 --> 12]
     */
    public void setSourceSpeed(double speed) {
        this.m_sourceMotor.setVoltage(speed);
    }
    
    /**
     * Sets the speed of the amp motor
     * @param speed the speed in volts [0 --> 12]
     */
    public void setAmpSpeed(double speed) {
        this.m_ampMotor.setVoltage(speed);
    }

    /**
     * A method to check if the phase changed
     * @param phase the phase that is checked
     */
    public boolean checkState(Phase phase) {
        return m_currentPhase == phase;
    }

    /**
     * A function for requesting ground/source intake using LEDs
     * @param intakeType The requested intake type (1 is ground, 2 is source, 0 is nothing)
     */
    public void requestIntake(int intakeType) {
        // Set the requested intake based on the input
        // If you request the same intake twice the lights turn off
        requestIntakeType = requestIntakeType == intakeType ? 0 : intakeType;

        // Set the LED color to the requested intake
        if (this.m_currentPhase != Phase.NONE) {
            requestIntakeType = 0;
        }

        switch (requestIntakeType) {
            case 1 -> this.powerLEDs(LEDColor.PURPLE);
            case 2 -> this.powerLEDs(LEDColor.CYAN);
        }

        // Intake request lights
        SmartDashboard.putBoolean("Ground Request", requestIntakeType == 1);
        SmartDashboard.putBoolean("Source Request", requestIntakeType == 2);
    }

    /**
     * ground intake command (with lights)
     * @param speed the speed to run the ground intake at in volts [0 --> 12]
     * @return
     */
    public Command groundIntake(double speed) {
        return this.startEnd(
            () -> {
                this.setBeltSpeed(-speed);
            },

            () -> {
                this.setBeltSpeed(-speed * 0.75);
            })
            .until(() -> this.checkState(Phase.GROUND_PICKUP))
            .andThen(startEnd(
                () -> {
                this.setBeltSpeed(-speed * 0.75);
            },
            () -> {
                this.setBeltSpeed(0.0);
            }).until(() -> this.checkState(Phase.LOADED))
        );
    }

    /**
     * Intake command to be used during auto (no lights)
     */
    public Command groundIntakeAuto(double speed) {
        return this.startEnd(
            () -> {
                this.setBeltSpeed(-speed);
            },

            () -> {
                this.setBeltSpeed(-speed * 0.75);
            })
            .until(() -> this.checkState(Phase.GROUND_PICKUP))
            .andThen(startEnd(
                () -> {
                this.setBeltSpeed(-speed * 0.75);
            },
            () -> {
                this.setBeltSpeed(0.0);
            }).until(() -> this.checkState(Phase.LOADED))
        );
    }
    

    /**
     * source intake command
     * @param speed the speed to run the source intake at in volts [0 --> 12]
     * @return
     */
    public Command sourceIntake(double speed) {
        return this.startEnd(
            () -> {
                this.setBeltSpeed(speed);
                this.setSourceSpeed(-speed);
                this.setAmpSpeed(-speed);
            },

            () -> {
                this.setBeltSpeed(speed * 0.75);
                this.setSourceSpeed(-speed * 0.75);
                this.setAmpSpeed(-speed * 0.75);
            })
            .until(() -> this.checkState(Phase.SOURCE_INTAKE))
            .andThen(startEnd(
                () -> {
                this.setBeltSpeed(speed * 0.75);
                this.setSourceSpeed(-speed * 0.75);
                this.setAmpSpeed(-speed * 0.75);
            },
            () -> {
                this.setBeltSpeed(0.0);
                this.setSourceSpeed(0.0);
                this.setAmpSpeed(0.0);
            })
            .until(() -> this.checkState(Phase.LOADED))
            .andThen(this.m_elevator.moveToPositionCommand(ElevatorPositions.INTAKE))
        );
    }

    /**
     * ejecting a note onto the ground command
     * @param speed the speed to run the source intake at in volts [0 --> 12]
     * @return
     */
    public Command groundReleaseAuto(double speed) {
        return this.startEnd(
                () -> {
                this.setBeltSpeed(speed);
                this.setSourceSpeed(-speed);
                this.setAmpSpeed(-speed);
            },
            () -> {
                this.setBeltSpeed(speed);
                this.setSourceSpeed(-speed);
                this.setAmpSpeed(-speed);
            })
            .until(() -> this.checkState(Phase.NONE))
            .andThen(this.m_elevator.moveToPositionCommand(ElevatorPositions.INTAKE));
    }

    /**
     * score amp command
     * @param speed the speed to run the amp at in volts [0 --> 12]
     * @return
     */
    public Command scoreAmp(double speed) {
        return this.startEnd(
            () -> {
                this.setBeltSpeed(-speed);
                this.setSourceSpeed(speed);
                this.setAmpSpeed(-speed * 0.7);
            },

            () -> {
                this.setBeltSpeed(-speed);
                this.setSourceSpeed(speed);
                this.setAmpSpeed(-speed * 0.7);
            }
        )
        .until(() -> this.checkState(Phase.NONE))
        .andThen(
            this.runOnce(
                () -> {
                    this.setBeltSpeed(0);
                    this.setSourceSpeed(0);
                    this.setAmpSpeed(0);
                }
            )
            .andThen(this.m_elevator.moveToPositionCommand(ElevatorPositions.INTAKE))
            .beforeStarting(new WaitCommand(0.25))
        );
    }

    /**
     * score speaker command
     * @param speed the speed to run the speaker at in volts [0 --> 12]
     * @return
     */
    public Command scoreSpeaker(double speed) {
        return this.startEnd(
            () -> {
                this.setBeltSpeed(-speed);
            },
            () -> {
                this.setBeltSpeed(0);
            }
        )
        .until(() -> this.checkState(Phase.NONE))
        .andThen(this.runOnce(() -> setWheelState(false)))
        .andThen(this.m_elevator.moveToPositionCommand(ElevatorPositions.INTAKE))
        .beforeStarting(new WaitCommand(0.4))
        .andThen(
            this.runOnce(
                () -> {
                    this.setSourceSpeed(0);
                    this.setAmpSpeed(0);
                }
            )
        );
    }

    /**
     * spin wheels command
     * @param speed the speed to run the wheels at in volts [0 --> 12]
     */
    public Command spinWheels(double speed) {
        System.out.println("CHARGING SHOOTER...");

        if (!areWheelsCharged) {
            return this.runOnce(
            () -> {
                this.setSourceSpeed(speed);
                this.setAmpSpeed(speed);
            }
            )
        .andThen(new WaitCommand(0.5)).andThen(this.runOnce(() -> setWheelState(true))).andThen(readyToShoot());
        }
        else {
            return this.runOnce(
            () -> {
                this.setSourceSpeed(speed);
                this.setAmpSpeed(speed);
            }
            );
        }
    }

    /**
     * Stop the mechanism from running
     * @return
     */
    public Command stopMechanism() {
        return runOnce(() -> {
          this.setBeltSpeed(0);
          this.setSourceSpeed(0);
          this.setAmpSpeed(0);
          this.setWheelState(false);
        });
    }

    /**
     * A command for confirming an intake
     */
    public Command confirmIntake() {
        return Commands.sequence(
            this.runOnce(() -> this.powerLEDs(LEDColor.GREEN)),
            new WaitCommand(0.1),
            this.runOnce(() -> this.powerLEDs(LEDColor.OFF)),
            new WaitCommand(0.1),
            this.runOnce(() -> this.powerLEDs(LEDColor.GREEN)),
            new WaitCommand(0.1),
            this.runOnce(() -> this.powerLEDs(LEDColor.OFF)),
            new WaitCommand(0.1),
            this.runOnce(() -> this.powerLEDs(LEDColor.GREEN))
        );
    }

    /**
     * Checks whether the wheels have been spun up
     */
    public boolean isCharged() {
        return areWheelsCharged;
    }

    /**
     * A command for confirming that the wheels are charged
     */
    public Command readyToShoot() {
        System.out.println("SHOOTER CHARGED");
        return Commands.runOnce(() -> this.powerLEDs(LEDColor.WHITE));
    }

    /**
     * Set whether the speaker wheels should be considered ready
     * @param state the wheel state to be set
     */
    public void setWheelState(boolean state) {
        areWheelsCharged = state;
        SmartDashboard.putBoolean("Wheels Charged", isCharged());
    }

    /**
     * A command for turning off all the LEDs
     */
    public Command lightsOff() {
        return Commands.runOnce(() -> this.powerLEDs(LEDColor.OFF));
    }

    /**
     * A function for changing the color of the LEDs using a string
     * @param colour the desired color name
     */
    public void powerLEDs(LEDColor color) {
        this.m_LEDs.setLEDs(color);
    }

    public void shotLights(boolean willHit) {
        if (willHit) {
            this.powerLEDs(LEDColor.GREEN);
        }
        else {
            this.powerLEDs(LEDColor.RED);
        }
    }

    public void periodic() {
        this.updatePhase();
        SmartDashboard.putString("Current Phase: ", this.m_currentPhase.name());
    }
}