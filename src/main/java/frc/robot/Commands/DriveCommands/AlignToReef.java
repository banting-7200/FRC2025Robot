package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ElevatorCommands.MoveElevator;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Field.Positions;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Vision.Limelight;
import java.util.ArrayList;
import java.util.Arrays;

public class AlignToReef extends Command {
  SwerveSubsystem swerve;
  ElevatorSubsystem elevator;
  Limelight limelight;
  Pose2d targetPose;
  Pose2d robotPose;
  boolean isRedAlliance;
  private final PIDController positionXController;
  private final PIDController positionYController;
  private final PIDController rotationController;

  public AlignToReef(
      SwerveSubsystem swerve,
      ElevatorSubsystem elevator,
      Limelight limelight,
      boolean isRedAlliance) {
    // Initialize Data //
    this.swerve = swerve;
    this.elevator = elevator;
    this.limelight = limelight;
    this.isRedAlliance = isRedAlliance;
    positionXController = new PIDController(4, 0, 0);
    positionYController = new PIDController(4, 0, 0);
    rotationController = new PIDController(0.12, 0, 0.01);

    rotationController.setTolerance(2, 4);
    positionXController.setTolerance(0.02, 0.04);
    positionYController.setTolerance(0.02, 0.04);

    addRequirements(swerve, elevator);
  }

  @Override
  public void initialize() {
    // Conditions //
    if (limelight.getTagCount() < 1) end(false);
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

    positionXController.setSetpoint(targetPose.getX());
    positionYController.setSetpoint(targetPose.getY());
    rotationController.setSetpoint(targetPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    robotPose = swerve.getPose();
    swerve.drive(
        new Translation2d(
            positionXController.calculate(robotPose.getX()),
            positionYController.calculate(robotPose.getY())),
        rotationController.calculate(swerve.getYaw().getDegrees()),
        true);
  }

  @Override
  public void end(boolean interrupted) {
    if (limelight.getTagCount() < 1) return;
    // Data //
    int aprilTag = limelight.getTagID();
    // Checks //
    if (isRedAlliance) aprilTag++;

    if (aprilTag % 2 == 1) new MoveElevator(elevator, Elevator.Positions.algaeOne).schedule();
    else new MoveElevator(elevator, Elevator.Positions.algaeTwo).schedule();
    System.out.println("DONE ALIGNING");
  }

  //   }

  @Override
  public boolean isFinished() {
    // Pose2d distance = targetPose.relativeTo(swerve.getPose());
    // if (Math.sqrt(distance.getX() * distance.getX() + distance.getY() * distance.getY()) < 0.1) {
    //   return true;
    // }
    if (new Translation2d(
                positionXController.calculate(robotPose.getX()),
                positionXController.calculate(robotPose.getY()))
            .getNorm()
        > .1) return false;
    if (!rotationController.atSetpoint()) return false;
    return true;
  }
}
