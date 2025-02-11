package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.*;
import java.io.File;

public class RobotContainer {
  private static RobotContainer instance;
  private XboxController mainController = new XboxController(Constants.Control.Main.port);
  private SwerveSubsystem drivebase;
  private Command driveFieldOrientedDirectAngle;

  public static EventLoop loop = new EventLoop();

  private RobotContainer() {
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    driveFieldOrientedDirectAngle =
        drivebase.driveCommand(
            () ->
                MathUtil.applyDeadband(
                    -mainController.getLeftY(), Constants.Control.Main.leftYDeadband),
            () ->
                MathUtil.applyDeadband(
                    -mainController.getLeftX(), Constants.Control.Main.leftXDeadband),
            () -> -mainController.getRightX(),
            () -> -mainController.getRightY());

    configureBindings();
  }

  public static RobotContainer getInstance() {
    if (instance == null) instance = new RobotContainer();
    return instance;
  }

  private void configureBindings() {

    BooleanEvent zeroDriveBase =
        new BooleanEvent(
            loop, () -> mainController.getRawButton(Constants.Control.Main.zeroSwerveDriveButton));
    zeroDriveBase.rising().ifHigh(() -> drivebase.zeroGyro());

    BooleanEvent enableCreepDrive =
        new BooleanEvent(
            loop, () -> mainController.getRawAxis(Constants.Control.Main.enableCreepDrive) > 0.5);
    enableCreepDrive.rising().ifHigh(() -> drivebase.setCreepDrive(true));
    enableCreepDrive.falling().ifHigh(() -> drivebase.setCreepDrive(false));
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  public void teleopPeriodic() {
    loop.poll();
  }
}
