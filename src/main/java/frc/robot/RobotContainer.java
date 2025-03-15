package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AlgaeCommands.IntakeAlgaeCommand;
import frc.robot.Commands.AlgaeCommands.MoveAlgaeArm;
import frc.robot.Commands.AlgaeCommands.OutputAlgaeCommand;
import frc.robot.Commands.DriveCommands.AlgaeObjectAlign;
import frc.robot.Commands.ElevatorCommands.MoveElevator;
import frc.robot.Commands.RumbleCommand;
import frc.robot.Constants.*;
import frc.robot.Subsystems.*;
import frc.robot.Vision.Limelight;
import frc.robot.Vision.PhotonVisionCamera;
import java.io.File;
import java.util.function.Supplier;

// import javax.smartcardio.CommandAPDU;

public class RobotContainer {
  private XboxController mainController = new XboxController(Control.Main.port);
  public Joystick buttonBox = new Joystick(Control.ButtonBox.port);
  private static SwerveSubsystem drivebase;
  private Command driveFieldOrientedDirectAngle;
  private static RobotContainer instance;

  // public CoralIntakeSubsystem coralController;
  public AlgaeIntakeSubsystem algaeController;
  public CageClimbSubsystem cageArm;
  public ElevatorSubsystem elevator;
  public Limelight limelight;
  public PhotonVisionCamera photonCam;

  public ShuffleboardSubsystem shuffle;
  SendableChooser<String> autos;

  public boolean isRobotInCoralMode;
  public boolean teleOpMode = false;

  public boolean atProcessorHeight = false;

  public final Supplier<double[]> joystickSquared =
      () -> {
        double[] d = drivebase.squareifyInput(mainController.getLeftX(), mainController.getLeftY());
        return /* isRedAliance.getAsBoolean() ? new double[] {d[0] * -1, d[1] * -1} : */ d;
      };

  // Supply right stick input, flipped dependant on alliance.
  public final Supplier<double[]> rightStickSupplier =
      () -> {
        double[] d = {mainController.getRightX() * -1, mainController.getRightY() * -1};
        return d;
      };

  //   private Commands commands = new Commands();
  //   public boolean isCoralMode;

  public static RobotContainer getInstance() {
    if (instance == null) instance = new RobotContainer();
    return instance;
  }

  //   private LightsSubsystem lights =
  //       new LightsSubsystem(Configurations.lightPort, Configurations.lightCount);

  private EventLoop loop = new EventLoop();
  private EventLoop testLoop = new EventLoop();
  private EventLoop swerveLoop = new EventLoop();

  private RobotContainer() {
    limelight = new Limelight("limelight");
    // coralController = new CoralIntakeSubsystem();
    algaeController = new AlgaeIntakeSubsystem();
    cageArm = new CageClimbSubsystem();
    elevator = new ElevatorSubsystem();
    shuffle = ShuffleboardSubsystem.getInstance();
    photonCam = new PhotonVisionCamera("algaeAlignCam");
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

    configBindings();
    initializeNamedCommands();
    initializeAutos();
  }

  public void coralConfigBindings() {
    BooleanEvent coralOutput = mainController.button(Control.Main.coralOutput, loop);

    coralOutput.rising().ifHigh(() -> algaeController.outputCoral());
    coralOutput.falling().ifHigh(() -> algaeController.stop());

    BooleanEvent coralArmOut = mainController.button(Control.Main.coralArmOut, loop);

    coralArmOut
        .rising()
        .ifHigh(() -> new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.down).schedule());
  }

  public void algaeConfigBindings() {
    BooleanEvent intakeAlgae = mainController.button(Control.Main.intake, loop);

    intakeAlgae
        .rising()
        .ifHigh(
            () ->
                new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.down)
                    .andThen(new IntakeAlgaeCommand(algaeController))
                    .andThen(new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.down - 10))
                    .schedule());

    BooleanEvent moveAlgaeToShoot = buttonBox.button(Control.ButtonBox.moveAlgaeToShoot, loop);

    moveAlgaeToShoot
        .rising()
        .ifHigh(() -> new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.shoot).schedule());

    // Output/Shoot Algae
    BooleanEvent outputAlgae = mainController.button(Control.Main.output, loop);

    outputAlgae
        .rising()
        .ifHigh(
            () ->
                new OutputAlgaeCommand(algaeController)
                    .andThen(new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.up))
                    .schedule());

    Trigger autoAlignToAlgae = new Trigger(() -> mainController.getRightTriggerAxis() > 0.5);
    autoAlignToAlgae.whileTrue(
        new AlgaeObjectAlign(
            drivebase, photonCam, joystickSquared, joystickSquared, algaeController));
  }

  public void swerveConfigBindings() {
    BooleanEvent zeroDriveBase =
        mainController.button(Constants.Control.Main.zeroSwerveDriveButton, swerveLoop);

    zeroDriveBase.rising().ifHigh(() -> drivebase.zeroGyro());

    BooleanEvent enableCreepDrive =
        mainController.axisGreaterThan(Constants.Control.Main.enableCreepDrive, 0.5, swerveLoop);

    enableCreepDrive.ifHigh(() -> drivebase.setCreepDrive(true));

    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  public void cageConfigBindings() {
    BooleanEvent cageUpEvent = buttonBox.button(Control.ButtonBox.moveAlgaeToShoot, loop);
    cageUpEvent.ifHigh(() -> cageArm.increaseSetpoint());

    BooleanEvent cageDownEvent = buttonBox.button(Control.ButtonBox.intake, loop);
    cageDownEvent.ifHigh(() -> cageArm.decreaseSetpoint());
  }

  public void elevatorConfigBindings() {

    BooleanEvent zeroElevator = buttonBox.button(Control.ButtonBox.reZeroElevator, loop);

    zeroElevator.rising().ifHigh(() -> elevator.zero());

    BooleanEvent elevatorFloorLevel = buttonBox.button(Control.ButtonBox.floorLevelButton, loop);

    elevatorFloorLevel
        .rising()
        .ifHigh(() -> new MoveElevator(elevator, Elevator.Positions.floorLevel).schedule());
    elevatorFloorLevel.rising().ifHigh(() -> atProcessorHeight = true);

    BooleanEvent elevatorProcessor = buttonBox.button(Control.ButtonBox.processorLevel, loop);
    elevatorProcessor
        .rising()
        .ifHigh(() -> new MoveElevator(elevator, Elevator.Positions.processor).schedule());
    elevatorProcessor.rising().ifHigh(() -> atProcessorHeight = true);

    BooleanEvent elevatorAlgaeOne = buttonBox.button(Control.ButtonBox.algaeLevel1, loop);

    elevatorAlgaeOne
        .rising()
        .ifHigh(() -> new MoveElevator(elevator, Elevator.Positions.algaeOne).schedule());

    elevatorAlgaeOne.rising().ifHigh(() -> atProcessorHeight = false);

    BooleanEvent elevatorAlgaeTwo = buttonBox.button(Control.ButtonBox.algaeLevel2, loop);
    elevatorAlgaeTwo
        .rising()
        .ifHigh(() -> new MoveElevator(elevator, Elevator.Positions.algaeTwo).schedule());

    elevatorAlgaeTwo.rising().ifHigh(() -> atProcessorHeight = false);

    BooleanEvent elevatorAlgaeNet = buttonBox.button(Control.ButtonBox.algaeNet, loop);
    elevatorAlgaeNet
        .rising()
        .ifHigh(
            () ->
                new MoveElevator(elevator, Elevator.Positions.top)
                    .alongWith(new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.shoot))
                    .schedule());

    elevatorAlgaeNet.rising().ifHigh(() -> atProcessorHeight = false);

    BooleanEvent moveUp = buttonBox.button(Control.ButtonBox.elevatorManualLift, loop);
    moveUp.ifHigh(() -> elevator.moveUp());

    BooleanEvent moveDown = buttonBox.button(Control.ButtonBox.elevatorManualFall, loop);

    moveDown.ifHigh(() -> elevator.moveDown());
  }

  public void configBindings() {
    Trigger rumbleTrigger = new Trigger(() -> algaeController.hasAlgae());
    rumbleTrigger.onTrue(
        new RumbleCommand(5, 1253, mainController).onlyIf(() -> teleOpMode == true));

    BooleanEvent flipMotor = buttonBox.button(Control.ButtonBox.inverseElevatorMotor, loop);

    flipMotor.rising().ifHigh(() -> elevator.flipMotor());
    // Bindings Methods //
    elevatorConfigBindings();
    coralConfigBindings();
    algaeConfigBindings();
    cageConfigBindings();
    swerveConfigBindings();

    // Trigger align = new Trigger(() -> mainController.getBButton());
    // align.whileTrue(
    //     new AlignToReef(
    //             drivebase, elevator, limelight,
    // isRedAlliance())
    //         .andThen(
    //             new TankDrive(drivebase, joystickSquared, joystickSquared)
    //                 .alongWith(new IntakeAlgaeCommand(algaeController))));
  }

  public void teleopPeriodic() {
    drivebase.setMediumDrive(elevator.getSetpoint() <= Elevator.Positions.mediumDriveZone);
    swerveLoop.poll();
    loop.poll();
    elevator.run();
    cageArm.run();
    if (limelight.tagCount() >= 1) {
      double orientation = drivebase.getYaw().getDegrees() + (isRedAlliance() ? 180 : 0);
      drivebase.addVisionMeasurement(limelight.getBotPose(orientation));
    }
  }

  public void robotPeriodic() {
    // System.out.println(
    //     "hasTarget :"
    //         + photonCam.hasTarget()
    //         + " targetYaw: "
    //         + photonCam.getTargetYaw()
    //         + " targetPitch: "
    //         + photonCam.getTargetPitch());
  }

  public void turnOffLimelight() {
    limelight.setLight(false);
  }

  public void turnOnLimelight() {
    limelight.setLight(true);
  }

  public void isTeleOp() {
    teleOpMode = true;
  }

  public void initializeAutos() {
    autos = new SendableChooser<>();
    autos.addOption("Centre 1 Algae", "Centre 1 Algae");
    autos.addOption("Test Auto", "Test Auto");
    autos.addOption("Centre 1.5 Algae", "Centre 1.5 Algae");
    autos.addOption("Comp 2 Auto", "Comp 2 Auto");
    shuffle.newAutoChooser(autos);
  }

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

  public Command getAutoCommand() {
    return drivebase.getAutonomousCommand(shuffle.getAuto());
  }

  public void updateShuffle() {
    shuffle.setTab("Driver");
    shuffle.setBoolean("Algae", algaeController.hasAlgae());
    // shuffle.setBoolean("Coral", coralController.hasCoral());

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
    shuffle.setLayout("Limelight", 1, 3);
    if (limelight.getTagCount() >= 1) {
      shuffle.setNumber("Tag Horizontal", limelight.getHorizontalMetres());
      shuffle.setNumber("Tag Distance", limelight.getDistanceMetres());
      shuffle.setNumber("Tag Rotation", limelight.getRotationDegrees());
    }
    shuffle.setLayout("Pose", 1, 3);
    Pose2d currentPose = drivebase.getPose();
    shuffle.setNumber("X", currentPose.getX());
    shuffle.setNumber("Y", currentPose.getY());
  }

  public void initializeNamedCommands() {
    // NamedCommands.registerCommand("Intake Coral", new IntakeCoralCommand(coralController));
    NamedCommands.registerCommand(
        "Intake Algae",
        new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.down)
            .andThen(new IntakeAlgaeCommand(algaeController)));
    // NamedCommands.registerCommand("Output Coral", new OutputCoralCommand(coralController));
    NamedCommands.registerCommand("Output Algae", new OutputAlgaeCommand(algaeController));
    NamedCommands.registerCommand(
        "Move Elevator To Coral One", new MoveElevator(elevator, Elevator.Positions.coralOne));
    NamedCommands.registerCommand(
        "Output Coral", new InstantCommand(() -> algaeController.outputCoral()));
    // NamedCommands.registerCommand(
    //     "Move Coral Arm To Output",
    //     new MoveCoralArm(coralController, CoralSystem.Positions.dropOff));
    // NamedCommands.registerCommand(
    //     "Move Coral Arm To Intake",
    //     new MoveCoralArm(coralController, CoralSystem.Positions.intake));
    // NamedCommands.registerCommand(
    //     "Move Coral Arm To Carry", new MoveCoralArm(coralController,
    // CoralSystem.Positions.carry));
    NamedCommands.registerCommand(
        "Move Algae Arm Down", new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.down));
    NamedCommands.registerCommand(
        "Move Algae Arm To Shoot", new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.shoot));
    NamedCommands.registerCommand(
        "Move Algae Arm Up", new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.up));
    NamedCommands.registerCommand(
        "Tilt Algae Arm", new MoveAlgaeArm(algaeController, AlgaeSystem.Positions.down - 10));
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
        "Move Elevator To Net", new MoveElevator(elevator, Elevator.Positions.top));
    NamedCommands.registerCommand(
        "Move Elevator To Floor Level", new MoveElevator(elevator, Elevator.Positions.floorLevel));
  }
}
