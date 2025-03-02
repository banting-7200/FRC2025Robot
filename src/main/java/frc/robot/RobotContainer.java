package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AlgaeCommands.IntakeAlgaeCommand;
import frc.robot.Commands.AlgaeCommands.MoveAlgaeArm;
import frc.robot.Commands.AlgaeCommands.OutputAlgaeCommand;
import frc.robot.Commands.ClimbCommands.MoveClimb;
import frc.robot.Commands.CoralCommands.IntakeCoralCommand;
import frc.robot.Commands.CoralCommands.MoveCoralArm;
import frc.robot.Commands.CoralCommands.OutputCoralCommand;
import frc.robot.Commands.DriveCommands.AlgaeObjectAlign;
import frc.robot.Commands.ElevatorCommands.MoveElevator;
import frc.robot.Commands.RumbleCommand;
import frc.robot.Constants.*;
import frc.robot.Controller.*;
import frc.robot.Controller.CommandButtonBox;
import frc.robot.Subsystems.*;
import frc.robot.Vision.Limelight;
import java.io.File;

// import javax.smartcardio.CommandAPDU;

public class RobotContainer {
  private CommandXBox mainController = new CommandXBox(Control.Main.port);
  private XboxController mainPhysicalController = (XboxController) mainController.getHID();
  public CommandButtonBox buttonBox = new CommandButtonBox(Control.ButtonBox.port);
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
                    -mainPhysicalController.getLeftX(), Constants.Control.Main.leftYDeadband),
            () ->
                MathUtil.applyDeadband(
                    mainPhysicalController.getLeftY(), Constants.Control.Main.leftXDeadband),
            () -> -mainPhysicalController.getRightX(),
            () -> -mainPhysicalController.getRightY());

    // configureBindings();
    configureNewBindings();
  }

  public void configureNewBindings() {
    // #region Swerve //
    mainController
        .zeroSwerveDrive()
        .onTrue(new InstantCommand(() -> drivebase.zeroGyroWithAlliance()));

    mainController
        .enableCreepDrive()
        .onTrue(new InstantCommand(() -> drivebase.setCreepDrive(true)))
        .onFalse(new InstantCommand(() -> drivebase.setCreepDrive(false)));

    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    // #endregion //

    // #endregion //
    // #region Algae //
    new Trigger(algaeController::hasAlgae)
        .onTrue(new RumbleCommand(1, 1253, mainPhysicalController));

    buttonBox
        .intake()
        .onTrue(
            new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.down)
                .andThen(new IntakeAlgaeCommand(algaeController)));

    buttonBox
        .coralManualRotateRight()
        .onTrue(new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.shoot));

    // Output/Shoot Algae

    buttonBox
        .output()
        .onTrue(
            new OutputAlgaeCommand(algaeController)
                .andThen(new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.shoot)));

    // #region Elevator //
    buttonBox.reZeroElevator().onTrue(new InstantCommand(() -> elevator.zero()));

    buttonBox.floorLevelButton().onTrue(new MoveElevator(elevator, Elevator.Positions.floorLevel));

    buttonBox.algaeLevel1().onTrue(new MoveElevator(elevator, Elevator.Positions.algaeOne));

    buttonBox.algaeLevel2().onTrue(new MoveElevator(elevator, Elevator.Positions.algaeTwo));

    buttonBox.algaeNet().onTrue(new MoveElevator(elevator, Elevator.Positions.top));
    // #endregion //
    // #region Cage //
    mainController.cageClimbUp().onTrue(new MoveClimb(cageArm, true));

    mainController.cageClimbDown().onTrue(new MoveClimb(cageArm, false));

    mainController.algaeAlign().onTrue(new AlgaeObjectAlign(mainController.algaeAlign().negate()));

    // BooleanEvent algaeAlign =
    //     new BooleanEvent(loop, () -> mainController.getRightTriggerAxis() > 0.5);

    // // Align until the button is released
    // algaeAlign.rising().ifHigh(() -> new AlgaeObjectAlign(algaeAlign.negate()).schedule());
    // #endregion //
  }

  public void configureBindings() {

    BooleanEvent switchTestMode =
        new BooleanEvent(
            testLoop,
            () -> mainPhysicalController.getRawButton(Constants.Control.Main.switchTestMode));

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
    shuffle.setNumber("Current", elevator.getCurrent());
    // shuffle.setBoolean("Upper Limit", elevator.topLimitSwitchPressed());
    // shuffle.setLayout("Limelight", 1, 3);
    // shuffle.setNumber("Tag Horizontal", limelight.getHorizontalMetres());
    // shuffle.setNumber("Tag Distance", limelight.getDistanceMetres());
    // shuffle.setNumber("Tag Rotation", limelight.getRotationDegrees());
  }
}
