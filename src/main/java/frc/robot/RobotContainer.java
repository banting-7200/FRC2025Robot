package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.AlgaeCommands.IntakeAlgaeCommand;
import frc.robot.Commands.AlgaeCommands.MoveAlgaeArm;
import frc.robot.Commands.AlgaeCommands.OutputAlgaeCommand;
import frc.robot.Commands.CoralCommands.IntakeCoralCommand;
import frc.robot.Commands.CoralCommands.MoveCoralArm;
import frc.robot.Commands.CoralCommands.OutputCoralCommand;
import frc.robot.Commands.ElevatorCommands.MoveElevator;
import frc.robot.Constants.*;
import frc.robot.Subsystems.*;
import frc.robot.Vision.Limelight;
import java.io.File;

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
  public Limelight limelight;

  public ShuffleboardSubsystem shuffle;

  public boolean isRobotInCoralMode;

  private int testMode = 0;

  //   private Commands commands = new Commands();
  //   public boolean isCoralMode;

  public static RobotContainer getInstance() {
    if (instance == null) instance = new RobotContainer();
    return instance;
  }

  private LightsSubsystem lights =
      new LightsSubsystem(Configurations.lightPort, Configurations.lightCount);

  private EventLoop loop = new EventLoop();
  private EventLoop testLoop = new EventLoop();
  private EventLoop swerveLoop = new EventLoop();

  private RobotContainer() {
    limelight = new Limelight("Limelight");
    coralController = new CoralIntakeSubsystem();
    algaeController = new AlgaeIntakeSubsystem();
    cageArm = new CageClimbSubsystem();
    elevator = new ElevatorSubsystem();
    shuffle = ShuffleboardSubsystem.getInstance();
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    // coralArm = new CoralIntakeSubsystem();

    NamedCommands.registerCommand("Intake Coral", new IntakeCoralCommand(coralController));
    NamedCommands.registerCommand("Intake Algae", new IntakeAlgaeCommand(algaeController));
    NamedCommands.registerCommand("Output Coral", new OutputCoralCommand(coralController));
    NamedCommands.registerCommand("Output Algae", new OutputAlgaeCommand(algaeController));
    NamedCommands.registerCommand(
        "Move Coral Arm To Output",
        new MoveCoralArm(coralController, CoralSystem.Positions.dropOff));
    NamedCommands.registerCommand(
        "Move Coral Arm To Intake",
        new MoveCoralArm(coralController, CoralSystem.Positions.intake));
    NamedCommands.registerCommand(
        "Move Coral Arm To Carry", new MoveCoralArm(coralController, CoralSystem.Positions.carry));
    NamedCommands.registerCommand(
        "Move Algae Arm Down", new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.down));
    NamedCommands.registerCommand(
        "Move Algae Arm Up", new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.up));
    NamedCommands.registerCommand(
        "Move Elevator To Coral 1", new MoveElevator(elevator, Elevator.Positions.coralOne));
    NamedCommands.registerCommand(
        "Move Elevator To Coral 2", new MoveElevator(elevator, Elevator.Positions.coralTwo));
    NamedCommands.registerCommand(
        "Move Elevator To Coral 3", new MoveElevator(elevator, Elevator.Positions.coralThree));
    NamedCommands.registerCommand(
        "Move Elevator To Coral 4", new MoveElevator(elevator, Elevator.Positions.coralFour));
    NamedCommands.registerCommand(
        "Move Elevator To Algae 1", new MoveElevator(elevator, Elevator.Positions.algaeOne));
    NamedCommands.registerCommand(
        "Move Elevator To Algae 2", new MoveElevator(elevator, Elevator.Positions.algaeTwo));
    NamedCommands.registerCommand(
        "Move Elevator To Floor Level", new MoveElevator(elevator, Elevator.Positions.floorLevel));

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

    // configureBindings();
    configureNewBindings();
  }

  public void configureNewBindings() {
    // #region Swerve //

    BooleanEvent zeroDriveBase =
        new BooleanEvent(
            swerveLoop,
            () -> mainController.getRawButton(Constants.Control.Main.zeroSwerveDriveButton));
    zeroDriveBase.rising().ifHigh(() -> drivebase.zeroGyroWithAlliance());

    BooleanEvent enableCreepDrive =
        new BooleanEvent(
            swerveLoop,
            () -> mainController.getRawAxis(Constants.Control.Main.enableCreepDrive) > 0.5);

    enableCreepDrive.rising().ifHigh(() -> drivebase.setCreepDrive(true));
    enableCreepDrive.falling().ifHigh(() -> drivebase.setCreepDrive(false));
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    // #endregion //

    // BooleanEvent elevatorUp =
    //     new BooleanEvent(loop, () ->
    // buttonBox.getRawButton(Control.ButtonBox.elevatorManualLift));
    // elevatorUp.ifHigh(() -> elevator.setMotorSpeed(Constants.Elevator.elevatorSpeed));
    // elevatorUp.falling().ifHigh(() -> elevator.stopMotor());

    // BooleanEvent elevatorDown =
    //     new BooleanEvent(loop, () ->
    // buttonBox.getRawButton(Control.ButtonBox.elevatorManualFall));
    // elevatorDown.ifHigh(() -> elevator.setMotorSpeed(-Constants.Elevator.elevatorSpeed));
    // elevatorDown.falling().ifHigh(() -> elevator.stopMotor());

    // #endregion //
    // #region Algae //
    BooleanEvent intakeAlgae =
        new BooleanEvent(loop, () -> buttonBox.getRawButton(Control.ButtonBox.intake));
    intakeAlgae
        .rising()
        .ifHigh(
            () ->
                new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.down)
                    .andThen(new IntakeAlgaeCommand(algaeController))
                    .schedule());

    BooleanEvent moveAlgaeToShoot =
        new BooleanEvent(
            loop, () -> buttonBox.getRawButton(Control.ButtonBox.coralManualRotateRight));
    moveAlgaeToShoot
        .rising()
        .ifHigh(() -> new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.shoot).schedule());

    // Output/Shoot Algae
    BooleanEvent outputAlgae =
        new BooleanEvent(loop, () -> buttonBox.getRawButton(Control.ButtonBox.output));
    outputAlgae
        .rising()
        .ifHigh(
            () ->
                new OutputAlgaeCommand(algaeController)
                    .andThen(new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.up))
                    .schedule());
    // #region Elevator //
    BooleanEvent zeroElevator =
        new BooleanEvent(loop, () -> buttonBox.getRawButton(Control.ButtonBox.reZeroElevator));
    zeroElevator.rising().ifHigh(() -> elevator.zero());

    BooleanEvent elevatorFloorLevel =
        new BooleanEvent(loop, () -> buttonBox.getRawButton(Control.ButtonBox.floorLevelButton));
    elevatorFloorLevel
        .rising()
        .ifHigh(() -> new MoveElevator(elevator, Elevator.Positions.floorLevel).schedule());

    BooleanEvent elevatorAlgaeOne =
        new BooleanEvent(loop, () -> buttonBox.getRawButton(Control.ButtonBox.algaeLevel1));

    elevatorAlgaeOne
        .rising()
        .ifHigh(() -> new MoveElevator(elevator, Elevator.Positions.algaeOne).schedule());

    BooleanEvent elevatorAlgaeTwo =
        new BooleanEvent(loop, () -> buttonBox.getRawButton(Control.ButtonBox.algaeLevel2));
    elevatorAlgaeTwo
        .rising()
        .ifHigh(() -> new MoveElevator(elevator, Elevator.Positions.algaeTwo).schedule());

    BooleanEvent elevatorAlgaeNet =
        new BooleanEvent(loop, () -> buttonBox.getRawButton(Control.ButtonBox.algaeNet));
    elevatorAlgaeNet
        .rising()
        .ifHigh(() -> new MoveElevator(elevator, Elevator.Positions.net).schedule());
    // #endregion //
    // #region Cage //
    BooleanEvent climbUp = new BooleanEvent(loop, () -> mainController.getPOV() == 0);
    climbUp.ifHigh(() -> cageArm.checkPOVAndMove(mainController.getPOV()));

    BooleanEvent climbDown = new BooleanEvent(loop, () -> mainController.getPOV() == 180);
    climbDown.ifHigh(() -> cageArm.checkPOVAndMove(mainController.getPOV()));
    // #endregion //
  }

  public void configureBindings() {

    BooleanEvent switchTestMode =
        new BooleanEvent(
            testLoop, () -> mainController.getRawButton(Constants.Control.Main.switchTestMode));

    switchTestMode.rising().ifHigh(() -> testMode++);

    
  }

  public void teleopPeriodic() {
    swerveLoop.poll();
    loop.poll();
    elevator.run();

  }

  public void turnOffLimelight() {
    limelight.setLight(false);
  }

  public void updateShuffle() {
    shuffle.setTab("Driver");
    shuffle.setBoolean("Algae", algaeController.hasAlgae());
    shuffle.setBoolean("Coral", coralController.hasCoral());

    shuffle.setTab("Status");
    shuffle.setLayout("Algae", 1, 3);
    shuffle.setBoolean("Has Algae", algaeController.hasAlgae());
    shuffle.setNumber("Angle", algaeController.getPosition());
    // shuffle.setLayout("Coral", 1, 3);
    // shuffle.setBoolean("Has Coral", coralController.hasCoral());
    // shuffle.setNumber("Angle", coralController.getPosition());
    shuffle.setLayout("Elevator", 1, 3);
    shuffle.setNumber("Height", elevator.getPosition());
    shuffle.setBoolean("Lower Limit", elevator.bottomLimitSwitchPressed());
    shuffle.setBoolean("Upper Limit", elevator.topLimitSwitchPressed());
    // shuffle.setLayout("Limelight", 1, 3);
    // shuffle.setNumber("Tag Horizontal", limelight.getHorizontalMetres());
    // shuffle.setNumber("Tag Distance", limelight.getDistanceMetres());
    // shuffle.setNumber("Tag Rotation", limelight.getRotationDegrees());
  }
}
