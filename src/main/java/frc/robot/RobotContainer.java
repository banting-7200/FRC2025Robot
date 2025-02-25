package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.*;
import frc.robot.Commands.AlgaeCommands.IntakeAlgaeCommand;
import frc.robot.Commands.AlgaeCommands.MoveAlgaeArm;
import frc.robot.Commands.CoralCommands.IntakeCoralCommand;
import frc.robot.Commands.CoralCommands.MoveCoralArm;
import frc.robot.Commands.CoralCommands.OutputCoralCommand;
import frc.robot.Constants.*;
import frc.robot.Subsystems.*;
import java.io.File;
import java.util.function.BooleanSupplier;

// import javax.smartcardio.CommandAPDU;

public class RobotContainer {
  private XboxController mainController = new XboxController(Control.Main.port);
  public Joystick buttonBox = new Joystick(Control.ButtonBox.port); 
  private SwerveSubsystem drivebase;
  private Command driveFieldOrientedDirectAngle;
  private static RobotContainer instance;

  public CoralIntakeSubsystem coralController;
  public AlgaeIntakeSubsystem algaeController;
  public CageClimbSubsystem cageArm;
  public ElevatorSubsystem elevator;
  public boolean isRobotInCoralMode;

  private void setRobotMode(boolean mode){
    isRobotInCoralMode = mode;
  }

  private boolean isRobotInCoralMode() {
    return isRobotInCoralMode;
  }

  BooleanSupplier robotHasCoral = this::isRobotInCoralMode;

  Trigger conditionTrigger = new Trigger(robotHasCoral);

  //   private Commands commands = new Commands();
  //   public boolean isCoralMode;

  public static RobotContainer getInstance() {
    if (instance == null) instance = new RobotContainer();
    return instance;
  }

  private LightsSubsystem lights =
      new LightsSubsystem(Configurations.lightPort, Configurations.lightCount);

  private EventLoop loop = new EventLoop();
  private EventLoop swerveLoop = new EventLoop();

  private RobotContainer() {
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    // coralArm = new CoralIntakeSubsystem();

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

  public void configureBindings() {

    BooleanEvent zeroDriveBase =
        new BooleanEvent(
            swerveLoop,
            () -> mainController.getRawButton(Constants.Control.Main.zeroSwerveDriveButton));
    zeroDriveBase.rising().ifHigh(() -> drivebase.zeroGyro());

    BooleanEvent enableCreepDrive =
        new BooleanEvent(
            swerveLoop,
            () -> mainController.getRawAxis(Constants.Control.Main.enableCreepDrive) > 0.5);
    enableCreepDrive.rising().ifHigh(() -> drivebase.setCreepDrive(true));
    enableCreepDrive.falling().ifHigh(() -> drivebase.setCreepDrive(false));
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    BooleanEvent switchGamePieceMode = new BooleanEvent(loop, () -> buttonBox.getRawButton(Control.ButtonBox.gamePieceSwitch));
    switchGamePieceMode.ifHigh(() -> setRobotMode(true));
    switchGamePieceMode.negate().ifHigh(() -> setRobotMode(false));

    //     // #region Algae related //
    //Intake algae: WILL NOT MOVE ELEVATOR
    conditionTrigger.negate().and(new JoystickButton(buttonBox, Control.ButtonBox.intake).onTrue(new MoveCoralArm(coralController, coralSystem.Positions.carry).andThen(new MoveAlgaeArm(algaeController, algaeSystem.Positions.armDown)).andThen(new IntakeAlgaeCommand(algaeController))));
    // Output algae command, will move arm back up
    conditionTrigger.negate().and(new JoystickButton(buttonBox, Control.ButtonBox.output).onTrue(new OutputCoralCommand(coralController).andThen(new MoveAlgaeArm(algaeController, algaeSystem.Positions.armUp))));
    // Move to floor level
    conditionTrigger.negate().and(new JoystickButton(buttonBox, Control.ButtonBox.floorLevelButton).onTrue(new MoveElevator(elevator, Elevator.Positions.floorLevel)));
    // Move to Algae elevator height: level 1
    conditionTrigger.negate().and(new JoystickButton(buttonBox, Control.ButtonBox.level1Button).onTrue(new MoveElevator(elevator, Elevator.Positions.algaeOne)));

    conditionTrigger.negate().and(new JoystickButton(buttonBox, Control.ButtonBox.level2Button).onTrue(new MoveElevator(elevator, Elevator.Positions.algaeTwo)));


    //     // #endregion //

    //      // //#region Coral related //
    // Coral Intake Command
    conditionTrigger.and(
        new JoystickButton(buttonBox, Control.ButtonBox.intake)
            .onTrue(
                new MoveAlgaeArm(algaeController, Constants.algaeSystem.Positions.armDown)
                    .alongWith(new MoveElevator(elevator, Elevator.Positions.floorLevel))
                    .andThen(new MoveCoralArm(coralController, coralSystem.Positions.intake))
                    .andThen(new IntakeCoralCommand(coralController))
                    .andThen(new MoveCoralArm(coralController, coralSystem.Positions.carry))
                    .andThen(new MoveElevator(elevator, Elevator.Positions.carry))
                    .alongWith(new MoveAlgaeArm(algaeController, algaeSystem.Positions.armUp))));

    // Coral Output Command
    conditionTrigger.and(
    new JoystickButton(buttonBox, Control.ButtonBox.output)
        .onTrue(new OutputCoralCommand(coralController)));

    // Ready output of coral at elevator height: Floor level
    conditionTrigger.and(
    new JoystickButton(buttonBox, Control.ButtonBox.floorLevelButton)
        .onTrue(
            new MoveElevator(elevator, Elevator.Positions.floorLevel)
                .alongWith(new MoveAlgaeArm(algaeController, algaeSystem.Positions.armDown))
                .andThen(new MoveCoralArm(coralController, coralSystem.Positions.dropOff))));

    // Ready output of coral at elevator height: level 1
    conditionTrigger.and(
    new JoystickButton(buttonBox, Control.ButtonBox.level1Button)
        .onTrue(
            new MoveElevator(elevator, Elevator.Positions.coralOne)
                .alongWith(new MoveAlgaeArm(algaeController, algaeSystem.Positions.armDown))
                .andThen(new MoveCoralArm(coralController, coralSystem.Positions.dropOff))));

    // Ready output of coral at elevator height: level 2
    conditionTrigger.and(
    new JoystickButton(buttonBox, Control.ButtonBox.level2Button)
        .onTrue(
            new MoveElevator(elevator, Elevator.Positions.coralTwo)
                .alongWith(new MoveAlgaeArm(algaeController, algaeSystem.Positions.armDown))
                .andThen(new MoveCoralArm(coralController, coralSystem.Positions.dropOff))));

    // Ready output of coral at elevator height: level 3
    conditionTrigger.and(
    new JoystickButton(buttonBox, Control.ButtonBox.level3Button)
        .onTrue(
            new MoveElevator(elevator, Elevator.Positions.coralThree)
                .alongWith(new MoveAlgaeArm(algaeController, algaeSystem.Positions.armDown))
                .andThen(new MoveCoralArm(coralController, coralSystem.Positions.dropOff))));

    // Ready output of coral at elevator height: level 4
    conditionTrigger.and(
    new JoystickButton(buttonBox, Control.ButtonBox.level4Button)
        .onTrue(
            new MoveElevator(elevator, Elevator.Positions.coralFour)
                .alongWith(new MoveAlgaeArm(algaeController, algaeSystem.Positions.armDown))
                .andThen(new MoveCoralArm(coralController, coralSystem.Positions.dropOff))));


    //      // #endregion //

  }

  public void teleopPeriodic() {
    swerveLoop.poll();
    loop.poll();
    // lights.run();

    // if (coralArm.hasCoral()) {
    //   lights.hasCoral();
    // } else if (algaeArm.hasAlgae()) {
    //   lights.hasAlgae();
    // } else {
    //   lights.hasNothing();
    // }
  }
}
