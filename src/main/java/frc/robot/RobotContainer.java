package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.Input;
import frc.robot.Subsystems.*;
import java.io.File;

public class RobotContainer {
  // Singleton //
  private static RobotContainer instance;
  ElevatorSubsystem elevatorSubsystem;
  private XboxController mainController = new XboxController(Constants.Control.Main.port);
  private Joystick buttonBox = new Joystick(0);
  private SwerveSubsystem drivebase;
  private Command driveFieldOrientedDirectAngle;

  private EventLoop loop = new EventLoop();

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
    // Elevator //

    BooleanEvent coralLevelOne =
        new BooleanEvent(loop, buttonBox.button(Input.coralOne, loop)).debounce(0.2);

    coralLevelOne
        .rising()
        .ifHigh(
            () -> {
              elevatorSubsystem.moveToCoralOne();
            });

    BooleanEvent coralLevelTwo =
        new BooleanEvent(loop, buttonBox.button(Input.coralTwo, loop)).debounce(0.2);

    coralLevelTwo
        .rising()
        .ifHigh(
            () -> {
              elevatorSubsystem.moveToCoralTwo();
            });

    BooleanEvent coralLevelThree =
        new BooleanEvent(loop, buttonBox.button(Input.coralThree, loop)).debounce(0.2);

    coralLevelThree
        .rising()
        .ifHigh(
            () -> {
              elevatorSubsystem.moveToCoralThree();
            });

    BooleanEvent coralLevelFour =
        new BooleanEvent(loop, buttonBox.button(Input.coralFour, loop)).debounce(0.2);

    coralLevelFour
        .rising()
        .ifHigh(
            () -> {
              elevatorSubsystem.moveToCoralFour();
            });

    BooleanEvent algaeLevelOne =
        new BooleanEvent(loop, buttonBox.button(Input.algaeOne, loop)).debounce(0.2);

    algaeLevelOne.rising().ifHigh(() -> {});

    BooleanEvent algaeLevelTwo =
        new BooleanEvent(loop, buttonBox.button(Input.algaeTwo, loop)).debounce(0.2);

    algaeLevelTwo.rising().ifHigh(() -> {});

    BooleanEvent intakeLevel =
        new BooleanEvent(loop, buttonBox.button(Input.intake, loop)).debounce(0.2);

    intakeLevel.rising().ifHigh(() -> {});

    BooleanEvent processorLevel =
        new BooleanEvent(loop, buttonBox.button(Input.processor, loop)).debounce(0.2);

    processorLevel.rising().ifHigh(() -> {});

    BooleanEvent netLevel = new BooleanEvent(loop, buttonBox.button(Input.net, loop));

    netLevel.rising().ifHigh(() -> {});
  }

  public void teleopPeriodic() {
    loop.poll();
  }
}
