package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraMode;

public class VisionUtils {

    /**
     * A function that converts the supplied Transform2d in robot relative coordinates 
     * into a Transform2d in field relative coordinates 
     * (flips the output because the camera is on the back).
     * @param objectTransform the transform of the object
     * @param heading the heading of the robot
     * @return the transform of the object in field coordinates
     */
    public static Transform2d robotToField(Transform2d objectTransform, double heading) {
        // Multiply the heading by PI/180 to convert to radians
        double sinHeading = Math.sin(heading * (Math.PI / 180));
        double cosHeading = Math.cos(heading * (Math.PI / 180));

        // Create field-relative coordinates using the heading and robot-relative coords
        double fieldX = objectTransform.getX() * cosHeading + objectTransform.getY() * -sinHeading;
        double fieldY = objectTransform.getX() * sinHeading + objectTransform.getY() * cosHeading;

        // Create the transform2d object
        Transform2d fieldTransform = new Transform2d(-fieldX, -fieldY, objectTransform.getRotation());

        return fieldTransform;
    }

    /**
     * A function that converts the supplied Transform2d in tag relative coordinates
     * into a Transform2d in field relative coordinates (this is different from robotToField 
     * because it doesn't flip the output).
     * @param objectTransform the transform of the object
     * @param heading the heading of the robot
     * @return the transform of the object in field coordinates
     */
    public static Transform2d tagToField(Transform2d objectTransform, double heading) {
      // Multiply the heading by PI/180 to convert to radians
      double sinHeading = Math.sin(heading * (Math.PI / 180));
      double cosHeading = Math.cos(heading * (Math.PI / 180));

      // Create field-relative coordinates using the heading and tag-relative coords
      double fieldX = objectTransform.getX() * cosHeading + objectTransform.getY() * -sinHeading;
      double fieldY = objectTransform.getX() * sinHeading + objectTransform.getY() * cosHeading;

      // Create the transform2d object
      Transform2d fieldTransform = new Transform2d(fieldX, fieldY, objectTransform.getRotation());

      return fieldTransform;
    }

    /**
     * A function for calculating a the movement towards a given apriltag's 
     * exact position (USED FOR THE AMP)
     * @param targetPose the pose of the apriltag
     * @param robotPose the pose of the robot
     * @param desiredOffset the desired offset from the apriltag
     * @return
     */
    public static Transform2d alignWithTagExact(Pose2d targetPose, Pose2d robotPose, Transform2d desiredOffset) {
      if (targetPose != null) {
        // Calculate the x and y commands based on the direction to the tag
        double xCommand = targetPose.getX() + desiredOffset.getX() - robotPose.getX();
        double yCommand = targetPose.getY() + desiredOffset.getY() - robotPose.getY();

        // Get the heading of the tag based on the camera mode (red/blue)
        Rotation2d tagRotation = Rotation2d.fromDegrees(targetPose.getRotation().getDegrees());
        // Robot heading
        Rotation2d robotRotation = robotPose.getRotation();
        // Commanded rotation
        double rotCommand = tagRotation.minus(robotRotation).getDegrees();

        // x axis command
        xCommand = (xCommand < 0) ? 
        Math.max(xCommand, -VisionConstants.kVisionSpeedLimit) : 
        Math.min(xCommand, VisionConstants.kVisionSpeedLimit);

        // y axis command
        yCommand = (yCommand < 0) ? 
        Math.max(yCommand, -VisionConstants.kVisionSpeedLimit) :
        Math.min(yCommand, VisionConstants.kVisionSpeedLimit);
        
        // Rotational command
        rotCommand = (rotCommand < 0) ? 
        Math.max(rotCommand, -VisionConstants.kVisionTurningLimit) :
        Math.min(rotCommand, VisionConstants.kVisionTurningLimit);

        // Set the x command to full speed if far enough
        if (Math.abs(xCommand) > VisionConstants.kTagSlowdownDistance) {
            xCommand = (xCommand < 0) ? 
            -VisionConstants.kVisionSpeedLimit : 
            VisionConstants.kVisionSpeedLimit;
        }

        // Set the y command to full speed if far enough
        if (Math.abs(yCommand) > VisionConstants.kTagSlowdownDistance) {
            yCommand = (yCommand < 0) ? 
            -VisionConstants.kVisionSpeedLimit : 
            VisionConstants.kVisionSpeedLimit;
        }

        // Check if robot is within acceptable boundaries
        boolean rotFinished = Math.abs(tagRotation.minus(robotRotation).getDegrees()) < VisionConstants.kTagRotationThreshold;
        boolean xFinished = Math.abs(targetPose.getX() + desiredOffset.getX() - robotPose.getX()) < VisionConstants.kTagDistanceThreshold;
        boolean yFinished = Math.abs(targetPose.getY() + desiredOffset.getY() - robotPose.getY()) < VisionConstants.kTagDistanceThreshold;

        if (rotFinished) { rotCommand = 0; }
        if (xFinished) { xCommand = 0; }
        // Do not zero the y command because that should be as exact as possible

        if (rotFinished && xFinished && yFinished) {
          return null;
        }
        else {
          // TODO: Maybe get rid of the * 0.3?
          return new Transform2d(xCommand, yCommand, Rotation2d.fromDegrees(rotCommand * 0.3));
        }
      }
      else {
        return null;
      }
    }

    /**
     * UNUSED (AND PROBABLY BROKEN) FUNCTION
     * A function for calculating a the movement towards a circle 
     * defined by a given apriltag (USED FOR THE SPEAKER)
     * @param targetPose the pose of the apriltag
     * @param robotPose the pose of the robot
     * @param desiredRadius how far you want to be from the target
     * @param tagAngleError the angle of the tag (0 is straight ahead)
     * @return
     */
    public static Transform2d alignWithTagRadial(Pose2d targetPose, Pose2d robotPose, double desiredRadius, double tagAngleError) {
        // Apriltag alignment code
        if (targetPose != null) {
          double distToTag = robotPose.getTranslation().getDistance(targetPose.getTranslation());
        
          // Calculate the x and y commands, based on whether the robot should travel away or towards the tag
          double xCommand = (distToTag > desiredRadius) ? (targetPose.getX() - robotPose.getX()) : (targetPose.getX() - robotPose.getX()) * -1;
          double yCommand = (distToTag > desiredRadius) ? (targetPose.getY() - robotPose.getY()) : (targetPose.getY() - robotPose.getY()) * -1;
          
          // Commanded rotation based on the tag angle
          double rotCommand = -tagAngleError;
  
          // x axis command
          xCommand = (xCommand < 0) ? 
          Math.max(xCommand, -VisionConstants.kVisionSpeedLimit) : 
          Math.min(xCommand, VisionConstants.kVisionSpeedLimit);
  
          // y axis command
          yCommand = (yCommand < 0) ? 
          Math.max(yCommand, -VisionConstants.kVisionSpeedLimit) :
          Math.min(yCommand, VisionConstants.kVisionSpeedLimit);
          
          // Rotational command
          rotCommand = (rotCommand < 0) ? 
          Math.max(rotCommand, -VisionConstants.kVisionTurningLimit) :
          Math.min(rotCommand, VisionConstants.kVisionTurningLimit);
  
          // Set the x command to full speed if far enough
          if (Math.abs(xCommand) > VisionConstants.kTagSlowdownDistance) {
              xCommand = (xCommand < 0) ? 
              -VisionConstants.kVisionSpeedLimit : 
              VisionConstants.kVisionSpeedLimit;
          }
  
          // Set the y command to full speed if far enough
          if (Math.abs(yCommand) > VisionConstants.kTagSlowdownDistance) {
              yCommand = (yCommand < 0) ? 
              -VisionConstants.kVisionSpeedLimit : 
              VisionConstants.kVisionSpeedLimit;
          }
  
          // Check if robot is within acceptable boundaries
          boolean rotFinished = Math.abs(Math.atan((targetPose.getY() - robotPose.getY()) / (targetPose.getX() - robotPose.getX()))) < VisionConstants.kTagRotationThreshold;
          boolean posFinished = Math.abs(distToTag - desiredRadius) < VisionConstants.kTagDistanceThreshold;
  
          if (rotFinished && posFinished) {
              return new Transform2d(0, 0, Rotation2d.fromDegrees(0));
          }
  
          // TODO: Maybe get rid of the * 0.3?
          return new Transform2d(0, 0, Rotation2d.fromDegrees(rotCommand * 0.3));
        }
        else {
          return new Transform2d(0, 0, Rotation2d.fromDegrees(0));
        }
    }
}
