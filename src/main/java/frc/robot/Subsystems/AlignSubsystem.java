package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.Field.Positions;
import java.util.ArrayList;
import java.util.Arrays;

public class AlignSubsystem {

  public static void alignToReef(SwerveSubsystem swerve) {
    Pose2d targetPose;
    Pose2d robotPose = swerve.getPose();

    targetPose =
        robotPose.nearest(
            new ArrayList<Pose2d>(
                Arrays.asList(
                    Positions.reefTwelveOclock,
                    Positions.reefTwoOclock,
                    Positions.reefFourOclock,
                    Positions.reefSixOclock,
                    Positions.reefEightOclock,
                    Positions.reefTenOclock)));

    System.out.println(String.format("Target: %2f, %2f", targetPose.getX(), targetPose.getY()));

    // rotate target pose 180 to face tag
    targetPose =
        new Pose2d(
            targetPose.getX(),
            targetPose.getY(),
            targetPose.getRotation().rotateBy(new Rotation2d(Math.PI)));
    // shift target pose back from reef
    targetPose =
        targetPose.transformBy(new Transform2d(Positions.backwardsOffset, 0, new Rotation2d()));
    // targetPose.plus(
    //     new Transform2d(
    //         new Translation2d(Positions.backwardsOffset, targetPose.getRotation()),
    //         new Rotation2d()));
    // drive to target pose

    swerve.driveToPose(targetPose).schedule();
  }
}
