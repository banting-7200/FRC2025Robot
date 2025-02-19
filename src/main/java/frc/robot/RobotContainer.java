package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Algae.Input;
import frc.robot.Subsystems.*;
import java.io.File;

public class RobotContainer {
  private static RobotContainer instance;
  private XboxController mainController = new XboxController(Constants.Control.Main.port);
  private SwerveSubsystem drivebase;
  private AlgaeIntakeSubsystem algaeIntakeSubsystem;
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

    // #region Subsystem Controls //
    BooleanEvent armToggleEvent =
        new BooleanEvent(loop, mainController.button(Input.toggleArm, loop)) // Arms //
            // debounce for more stability
            .debounce(0.2);

    // If button is pressed, toggle arm //
    armToggleEvent
        .rising()
        .ifHigh(
            () -> {
              algaeIntakeSubsystem.ArmToggle();
            });

    BooleanEvent intakeEvent =
        new BooleanEvent(loop, mainController.button(Input.intake, loop)).debounce(0.2);

    intakeEvent
        .rising()
        .ifHigh(
            () -> {
              algaeIntakeSubsystem.Intake();
            });

    BooleanEvent outputEvent =
        new BooleanEvent(loop, mainController.button(Input.output, loop)).debounce(0.2);

    outputEvent.rising().ifHigh(() -> {});
  }

  public void teleopPeriodic() {
    loop.poll();
  }
}
