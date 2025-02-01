package frc.robot.Vision;

import frc.robot.Vision.LimelightHelpers.PoseEstimate;

public class Limelight {
  private final String name;

  public Limelight(String name) {
    this.name = name;
  }

  public PoseEstimate getBotPose(double robotYaw) {
    // First, tell Limelight your robot's current orientation
    LimelightHelpers.SetRobotOrientation(name, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    return limelightMeasurement;
  }
}
