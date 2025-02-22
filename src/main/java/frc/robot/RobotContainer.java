package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Algae;
import frc.robot.Constants.CageConstants;
import frc.robot.Constants.Configurations;
import frc.robot.Constants.Control;
import frc.robot.Constants.Coral;
import frc.robot.Constants.Elevator;
import frc.robot.Subsystems.*;
import java.io.File;

public class RobotContainer {
  private static RobotContainer instance;
  private XboxController mainController = new XboxController(Control.Main.port);
  public Joystick buttonBox = new Joystick(Control.ButtonBox.port); // TODO: TUNE PORT //
  private SwerveSubsystem drivebase;
  private Command driveFieldOrientedDirectAngle;

  private CoralIntakeSubsystem coralArm = CoralIntakeSubsystem.getInstance();
  private AlgaeIntakeSubsystem algaeArm = AlgaeIntakeSubsystem.getInstance();
  private CageClimbSubsystem cageArm = CageClimbSubsystem.getInstance();
  private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

  private LightsSubsystem lights =
      new LightsSubsystem(Configurations.lightPort, Configurations.lightCount);

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

    // #region Intake/Output //
    // for coral/algae intake
    BooleanEvent intakeEvent = buttonBox.button(Coral.Input.intake, loop).debounce(0.2);

    intakeEvent
        .rising()
        .ifHigh(
            () -> {
              if (Elevator.mode) {
                algaeArm.Intake();
              } else {
                coralArm.intake();
              }
            });

    intakeEvent
        .falling()
        .ifHigh(
            () -> {
              if (Elevator.mode) {
                algaeArm.StopIntake();
              } else {
                coralArm.stopIntake();
              }
            });

    // for coral/algae output
    BooleanEvent outputEvent = buttonBox.button(Coral.Input.output, loop).debounce(0.2);

    outputEvent
        .rising()
        .ifHigh(
            () -> {
              if (Elevator.mode) { // If Algae Mode
                algaeArm.Output();
              } else { // If Coral Mode
                coralArm.output();
              }
            });

    outputEvent
        .falling()
        .ifHigh(
            () -> {
              if (Elevator.mode) { // If Algae Mode
                algaeArm.StopIntake();
              } else { // If Coral Mode
                coralArm.stopIntake();
              }
            });
    // #region Coral //
    BooleanEvent coralArticulateUp =
        buttonBox.button(Coral.Input.armArticulateUp, loop).debounce(0.2);

    coralArticulateUp.ifHigh(
        () -> {
          coralArm.coralArticulateUp();
        });

    BooleanEvent coralArticulateDown =
        buttonBox.button(Coral.Input.armArticulateDown, loop).debounce(0.2);

    coralArticulateDown.ifHigh(
        () -> {
          coralArm.coralArticulateDown();
        });
    // #endregion //
    // #region Algae //
    BooleanEvent algaeArticulateUp =
        buttonBox.button(Algae.Input.armArticulateUp, loop).debounce(0.2);

    algaeArticulateUp.ifHigh(
        () -> {
          coralArm.coralArticulateUp();
        });

    BooleanEvent algaeArticulateDown =
        buttonBox.button(Algae.Input.armArticulateDown, loop).debounce(0.2);

    algaeArticulateDown.ifHigh(
        () -> {
          algaeArm.AlgaeArmArticulateDown();
        });
    // #endregion //
    // #region Cage //
    BooleanEvent toggleCageEvent =
        mainController.button(CageConstants.Input.toggleCage, loop).debounce(0.2);

    toggleCageEvent
        .rising()
        .ifHigh(
            () -> {
              cageArm.armToggle();
            });
    // #endregion
    // #region Elevator //
    BooleanEvent goFloorLevel = buttonBox.button(Elevator.Input.floorLevel, loop).debounce(0.2);
    goFloorLevel.rising().ifHigh(() -> elevator.moveToFloor());

    BooleanEvent goLevelOne = buttonBox.button(Elevator.Input.levelOne, loop).debounce(0.2);
    goLevelOne
        .rising()
        .ifHigh(
            () -> {
              if (Elevator.mode) {
                elevator.moveToAlgaeOne();
                algaeArm.ArmDown();
              } else {
                elevator.moveToCoralOne();
                coralArm.moveToDropoff();
              }
            });

    BooleanEvent goLevelTwo = buttonBox.button(Elevator.Input.levelTwo, loop).debounce(0.2);
    goLevelTwo
        .rising()
        .ifHigh(
            () -> {
              if (Elevator.mode) {
                elevator.moveToAlgaeTwo();
                algaeArm.ArmDown();
              } else elevator.moveToCoralTwo();
            });

    BooleanEvent goLevelThree = buttonBox.button(Elevator.Input.levelThree, loop).debounce(0.2);
    goLevelThree
        .rising()
        .ifHigh(
            () -> {
              elevator.moveToCoralThree();
              algaeArm.ArmDown();
            });

    BooleanEvent goLevelFour = buttonBox.button(Elevator.Input.levelFour, loop).debounce(0.2);
    goLevelFour
        .rising()
        .ifHigh(
            () -> {
              elevator.moveToCoralFour();
              algaeArm.ArmDown();
            });
    // #endregion //
  }

  public void teleopPeriodic() {
    loop.poll();
    lights.run();

    if (coralArm.hasCoral()) {
      lights.hasCoral();
    } else if (algaeArm.hasAlgae()) {
      lights.hasAlgae();
    } else {
      lights.hasNothing();
    }
  }
}
