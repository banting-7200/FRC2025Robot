package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ElevatorCommands.MoveElevator;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Field.Positions;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

public class AlignToReef extends Command {
  SwerveSubsystem swerve;
  ElevatorSubsystem elevator;
  RobotContainer robot = RobotContainer.getInstance();
  Pose2d targetPose;
  Pose2d robotPose;
  Command runningCommand;
  Supplier<double[]> leftJoystick;
  Supplier<double[]> rightJoystick;

  public AlignToReef(
      SwerveSubsystem swerve,
      ElevatorSubsystem elevator,
      Supplier<double[]> leftJoystick,
      Supplier<double[]> rightJoystick) {
    // Initialize Data //
    this.swerve = swerve;
    this.elevator = elevator;
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
  }

  @Override
  public void initialize() {
    // Initialize Data //
    this.robotPose = swerve.getPose();
    this.targetPose =
        robotPose.nearest(
            new ArrayList<Pose2d>(
                Arrays.asList(
                    Positions.reefTwelveOclock,
                    Positions.reefTwoOclock,
                    Positions.reefFourOclock,
                    Positions.reefSixOclock,
                    Positions.reefEightOclock,
                    Positions.reefTenOclock)));
    // Debug //
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

    runningCommand = swerve.driveToPose(targetPose);
    runningCommand.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    runningCommand.cancel();

    int aprilTag = robot.limelight.getTagID();

    if (robot.isRedAlliance()) aprilTag++;

    if (aprilTag % 2 == 1) new MoveElevator(elevator, Elevator.Positions.algaeOne);
    else new MoveElevator(elevator, Elevator.Positions.algaeTwo);
  }

  @Override
  public boolean isFinished() {
    return runningCommand.isFinished();
  }
}
