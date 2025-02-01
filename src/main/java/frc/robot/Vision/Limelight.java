package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Vision.LimelightHelpers.PoseEstimate;

public class Limelight {
  private final String name;

  public Limelight(String name) {
    this.name = name;
  }

  public Pose2d getBotPose(double robotYaw) {
    // First, tell Limelight your robot's current orientation
    LimelightHelpers.SetRobotOrientation(name, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    return limelightMeasurement.pose;
  }

  public int tagCount() {
    return LimelightHelpers.getTargetCount(name);
  }
}
