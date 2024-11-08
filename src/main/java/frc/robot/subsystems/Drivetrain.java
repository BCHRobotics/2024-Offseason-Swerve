// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.Constants.VisionConstants.CameraMode;
import frc.utils.SwerveUtils;
import frc.utils.VisionUtils;
import frc.utils.devices.Camera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  // A percentage value (0-1) for the linear speed of the robot
  private double m_maxSpeed = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private boolean m_slowMode = false;
  private boolean m_fastMode = false;

  // If you switch the camera you have to change the name property of this
  private final Camera m_noteCamera = new Camera(VisionConstants.kNoteCameraName); // These names might need to be changed
  private final Camera m_tagCamera = new Camera(VisionConstants.kTagCameraName); // this too

  public boolean isRedAlliance;

  private boolean isVisionAuto;

  private DriveModes driveMode = DriveModes.MANUAL;

  public boolean onTarget;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    this.initializeAuto();
  }

  // Very important drive mode functions that make everything work
  public DriveModes getDriveMode() {
    return driveMode;
  }

  public void setDriveMode(DriveModes modeToSet) {
    driveMode = modeToSet;
  }

  @Override
  public void periodic() {
    // temporary code for storing trajectory predictions to be used by commands
    
    // define speaker target pose
    Pose2d speakerPose = VisionUtils.getPose(CameraMode.SPEAKER, isRedAlliance);
    if (speakerPose != null) {
      boolean shotPred = VisionUtils.isReadyToShoot(6.95, 0.5, 0.95, this.m_odometry.getPoseMeters().getRotation().getDegrees(), speakerPose, getPose(), getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
      Mechanism.getInstance().shotLights(shotPred);
      onTarget = shotPred;
    }
    else {
      onTarget = false;
    }

    // Refresh the data gathered by the camera
    m_noteCamera.refreshResult();
    m_tagCamera.refreshResult();

    // Set the max speed of the bot
    setSpeedPercent();

    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Print debug values to smartDashboard
    this.printToDashboard();

    // ---- Updating Apriltag Positions ---- //
    // Loop through all apriltag ids
    for (int i = 0; i < VisionUtils.apriltagCount; i++) {
      // Check to see if the current id is flagged
      // This means we are keeping track of it
      boolean isFlagged = false;
      for (int j = 0; j < VisionUtils.flaggedTags.length; j++) {
        if (VisionUtils.flaggedTags[j] == i) {
          isFlagged = true;
          break;
        }
      }

      // If the current tag isn't flagged it's useless to us
      // So skip to the next id
      if (!isFlagged) { continue; }
      // all code below THIS POINT will NOT RUN if the tag isn't flagged

      if (m_tagCamera.hasTargetOfId(i)) {
        VisionUtils.tagPositions[i] = m_tagCamera.getApriltagPose(getPose(), this.m_odometry.getPoseMeters().getRotation().getDegrees(), i, VisionUtils.getHeadingFromTagIndex(i, isRedAlliance));
      }
    }

    // TODO: THIS CODE WILL BE DELETED ONCE TESTED
    // // Update the amp target pose
    // int desiredTagId = isRedAlliance ? 5 : 6; // Which amp tag to target (blue or red)
    // if (m_tagCamera.hasTargetOfId(desiredTagId)) {
    //   ampTargetPose = m_tagCamera.getApriltagPose(getPose(), this.m_odometry.getPoseMeters().getRotation().getDegrees(), desiredTagId, CameraMode.AMP.getHeading(isRedAlliance));
    // }

    // // Update the speaker target pose
    // desiredTagId = isRedAlliance ? 4 : 7; // Which speaker tag to target (blue or red)
    // if (m_tagCamera.hasTargetOfId(desiredTagId)) {
    //   speakerTargetPose = m_tagCamera.getApriltagPose(getPose(), this.m_odometry.getPoseMeters().getRotation().getDegrees(), desiredTagId, CameraMode.SPEAKER.getHeading(isRedAlliance));
    // }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Lying to pathplanner
   */
  public Pose2d getAssistedPose() {
    // Here we are interpreting the angle from the robot to the note as a distance (technically wrong but it works out anyways)
    // We are using this distance to construct a robot-relative offset to the note (again, this is wrong but it works)
    Transform2d robotNoteOffset = new Transform2d(0, -m_noteCamera.getAngleError() * 0.07, new Rotation2d());
    // Convert to field relative coords using rotation matrix
    Transform2d fieldNoteOffset = VisionUtils.tagToField(robotNoteOffset, getHeading());
    // Using tagToField() here because the note is on the front of the bot (I am aware the functions need renaming)

    if (isVisionAuto) {
      return m_odometry.getPoseMeters().plus(fieldNoteOffset);
    }
    else {
      return m_odometry.getPoseMeters();
    }
  }

  public void autoVision(boolean input) {
    isVisionAuto = input;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /*
   * A function for setting the alliance of the robot
   */
  public void setAlliance(boolean isRed) {
    isRedAlliance = isRed;
  }
  
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param maxSpeed      A 0-1 multiplier for the x and y speed of the robot.
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;
    
    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // Some small number to avoid floating-point errors with equality checking
          // Keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    /*
     * Convert the commanded speeds into the correct units for the drivetrain,
     * using the given max speed
     */
    double xSpeedDelivered = xSpeedCommanded * m_maxSpeed;
    double ySpeedDelivered = ySpeedCommanded * m_maxSpeed;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-this.m_odometry.getPoseMeters().getRotation().getDegrees() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
 
    this.setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   * This does not set the brake mode of the motors.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, m_maxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Gets the swerve ModuleStates.
   *
   * @return The current SwerveModule states.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Sets the idle states of the SparkMAX motors.
   *
   * @param mode the mode to set the states to (0 is coast, 1 is brake)
   */
  public void setIdleStates(int mode) {
    m_frontLeft.setIdle(mode);
    m_rearLeft.setIdle(mode);
    m_frontRight.setIdle(mode);
    m_rearRight.setIdle(mode);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    // Change this after
    resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -Infinity to Infinity
   */
  public double getHeading() {
    /* I'm multiplying the navx heading by -1 
    * because WPILib uses CCW as the positive direction
    * and NavX uses CW as the positive direction
    */ 
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }

  /**
   * Enables and disables slow mode.
   *
   * @param mode Whether to enable slow mode on or off.
   */
  public void setSlowMode(boolean mode) {
    this.m_slowMode = mode;
    setSpeedPercent();
  }

  /**
   * Enables and disables fast mode.
   *
   * @param mode Whether to enable fast mode on or off.
   */
  public void setFastMode(boolean mode) {
    this.m_fastMode = mode;
    setSpeedPercent();
  }

  /**
   * Sets the speed of the robot to a desired m/s value
   *
   * @param percent The desired speed in metres per second
   */
  public void setSpeedPercent() {
    if (m_slowMode) {
      m_maxSpeed = DriveConstants.kMinSpeedMetersPerSecond;
    } else if (m_fastMode) {
      m_maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
    } else {
      m_maxSpeed = DriveConstants.a;
    }
  }

  /**
   * Initializes the auto using PathPlannerLib.
   */
  public void initializeAuto() {
    AutoBuilder.configureHolonomic(
          this::getAssistedPose, 
          this::resetOdometry, 
          this::getChassisSpeeds, 
          this::setChassisSpeeds,
          new HolonomicPathFollowerConfig( 
                  new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                  new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
                  AutoConstants.kMaxSpeedMetersPerSecond,
                  AutoConstants.kDriveBase, // Distance from robot center to furthest module
                  new ReplanningConfig() 
          ),
          () -> {
              Optional<Alliance> alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
          },
          this   // Reference to this subsystem to set requirements
        );
  }
  
  /**
   * Sets the speed of the robot chassis.
   * @param speed The new chassis speed.
   */
  public void setChassisSpeeds(ChassisSpeeds speed) {
    this.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speed));
  }

  /**
   * Gets the speed of the robot chassis.
   * @return The current chassis speed.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(this.getModuleStates());
  }

  /** Prints all values to the dashboard. */
  public void printToDashboard() {

    // Speed
    // SmartDashboard.putNumber("Vertical Speed", this.getChassisSpeeds().vyMetersPerSecond); // Field relative horizontal speed
    // SmartDashboard.putNumber("Horizontal Speed", this.getChassisSpeeds().vxMetersPerSecond); // Field relative vertical speed
    // SmartDashboard.putNumber("Turn Speed", this.getChassisSpeeds().omegaRadiansPerSecond); // Field relative turn speed
    SmartDashboard.putNumber("Current Speed Percentage", m_maxSpeed); // Commanded speed multiplier [0 --> 1]

    // Position
    SmartDashboard.putNumber("X Position", this.getPose().getX());
    SmartDashboard.putNumber("Y Position", this.getPose().getY());
    // SmartDashboard.putNumber("Gyro Heading: ", this.getHeading());
    SmartDashboard.putNumber("Odometry Heading: ", this.m_odometry.getPoseMeters().getRotation().getDegrees());

    // Slew rate filter variables
    // SmartDashboard.putNumber("slewCurrentRotation: ", m_currentRotation);
    // SmartDashboard.putNumber("slewCurrentTranslationDirection: ", m_currentTranslationDir);
    // SmartDashboard.putNumber("slewCurrentTranslationMagnitude: ", m_currentTranslationMag);

    // Encoder values
    // SmartDashboard.putNumber("Front left Encoder", m_frontLeft.getVel());
    // SmartDashboard.putString("Front right Encoder", m_frontRight.getState().toString());
    // SmartDashboard.putString("Rear left Encoder", m_rearLeft.getState().toString());
    // SmartDashboard.putString("Rear right Encoder", m_rearRight.getState().toString());

    // Apriltag target location/rotation for amp (field relative space)
    // if (ampTargetPose != null) {
    //   SmartDashboard.putNumber("Target X", ampTargetPose.getX());
    //   SmartDashboard.putNumber("Target Y", ampTargetPose.getY());

    //   SmartDashboard.putNumber("Target Rotation", ampTargetPose.getRotation().getDegrees());
    // }

    // Apriltag target location/rotation for speaker (field relative space)
    if (VisionUtils.getPose(CameraMode.SPEAKER, isRedAlliance) != null) {
      double xDist = new Transform2d(getPose(), VisionUtils.getPose(CameraMode.SPEAKER, isRedAlliance)).getX();
      double yDist = new Transform2d(getPose(), VisionUtils.getPose(CameraMode.SPEAKER, isRedAlliance)).getY();
      SmartDashboard.putNumber("DIST", Math.sqrt(xDist * xDist + yDist * yDist));
      // SmartDashboard.putNumber("Target X", speakerTargetPose.getX());
      // SmartDashboard.putNumber("Target Y", speakerTargetPose.getY());

      // SmartDashboard.putNumber("Target Rotation", speakerTargetPose.getRotation().getDegrees());
    }

    SmartDashboard.putNumber("TEST", getChassisSpeeds().vxMetersPerSecond);

    // Do the cameras have targets?
    SmartDashboard.putBoolean("Note Cam", m_noteCamera.getResult().hasTargets());
    SmartDashboard.putBoolean("Tag Cam", m_tagCamera.getResult().hasTargets());

    // Are the cameras connected?
    SmartDashboard.putBoolean("Note Cam Connected", m_noteCamera.isConnected());
    SmartDashboard.putBoolean("Tag Cam Connected", m_tagCamera.isConnected());
  }
}
