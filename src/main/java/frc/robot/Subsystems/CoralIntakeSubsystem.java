// Directory //
package frc.robot.Subsystems;

// Imports //
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Coral.*;
import frc.robot.RobotContainer;

// Subsystem //
public class CoralIntakeSubsystem {
  // Singleton //
  public static CoralIntakeSubsystem instance;
  // Input //
  Joystick buttonBox = RobotContainer.getInstance().buttonBox;
  // Subsystems //
  AlgaeIntakeSubsystem algaeArm;
  // Motors //
  SparkMax pivotMotor;
  SparkMax intakeMotor;
  // Motor Data //
  SparkAbsoluteEncoder encoder;
  SparkClosedLoopController pidController;
  SparkMaxConfig config;
  // Limit Switches //
  SparkLimitSwitch pivotFLimitSwitch;
  SparkLimitSwitch pivotRLimitSwitch;
  SparkLimitSwitch intakeRLimitSwitch;
  // Data //
  private double setPoint = Positions.carry;

  // Constructor //
  public CoralIntakeSubsystem() {
    // Init Motors //
    pivotMotor = new SparkMax(DeviceIDs.pivotMotor, MotorType.kBrushless);
    intakeMotor = new SparkMax(DeviceIDs.intakeMotor, MotorType.kBrushless);
    // Init Config //
    config = new SparkMaxConfig();
    // Init Encoder //
    encoder = pivotMotor.getAbsoluteEncoder();
    // Init PID //
    pidController = pivotMotor.getClosedLoopController();
    // Setup Config //
    config.inverted(true).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(360).velocityConversionFactor(1);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(PID.p, PID.i, PID.d);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Limit Switches //
    pivotFLimitSwitch = pivotMotor.getForwardLimitSwitch();
    pivotRLimitSwitch = pivotMotor.getReverseLimitSwitch();
    intakeRLimitSwitch = intakeMotor.getForwardLimitSwitch();
    // Get Algae Intake Subsystem //
    algaeArm = AlgaeIntakeSubsystem.getInstance();
    // Singleton //
    instance = this;
  }

  public static synchronized CoralIntakeSubsystem getInstance() {
    // If no instance previously made then create one //
    if (instance == null) return new CoralIntakeSubsystem();
    // Return Instance //
    return instance;
  }

  // #region Base Methods //
  public void moveToPosition(double setPoint) {
    this.setPoint = setPoint;
  }

  public double getSetPoint() {
    return setPoint;
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public void run() {

    // Input //
    boolean armArticulateDownPressed = buttonBox.getRawButton(Input.armArticulateDown);
    boolean armArticulateUpPressed = buttonBox.getRawButton(Input.armArticulateUp);
    // Checks //
    if (armArticulateDownPressed) coralArticulateDown();
    if (armArticulateUpPressed) coralArticulateUp();

    if (encoder.getPosition() > Positions.intake + 2
        && encoder.getPosition() < Positions.dropOff - 2) {
      pidController.setReference(setPoint, ControlType.kPosition);
    } else {
      pivotMotor.stopMotor();
    }
  }

  public boolean isAtCarryPosition() {
    double pivotPosition = getPosition();

    return pivotPosition >= Positions.carry - Positions.safetyRange / 2
        && pivotPosition <= Positions.carry + Positions.safetyRange / 2;
  }

  // #endregion
  // #region Level Methods //
  /** Moves the Coral arm to the intake position */
  public void moveToIntake() { // right from our view
    // Wait Until Arm is Down //
    new WaitUntilCommand(algaeArm::IsDown)
        .andThen(
            () -> { // Move The Coral Arm to Intake Position //
              moveToPosition(Positions.intake);
            });
  }

  public void moveToCarry() { // middle
    moveToPosition(Positions.carry);
  }

  public void moveToDropoff() { // Left from our view
    // Wait Until Arm is Down //
    new WaitUntilCommand(algaeArm::IsDown)
        .andThen(
            () -> { // Move The Coral Arm to Intake Position //
              moveToPosition(Positions.dropOff);
            });
  }

  public boolean hasCoral() {
    return intakeRLimitSwitch.isPressed();
  }

  // jesse we need to cook green bananas on chewsday with the fake sugar clrystals
  // #endregion
  // #region Action Methods //
  public void intake() {
    intakeMotor.set(-MotorSpeeds.intakeSpeed);

    new WaitUntilCommand(intakeRLimitSwitch::isPressed)
        .andThen(
            () -> {
              stopIntake();
            });
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void output() {
    intakeMotor.set(MotorSpeeds.outputSpeed);
  }

  // #endregion

  public void coralArticulateUp() {
    // Conditions //
    if (pivotFLimitSwitch.isPressed()) return;
    // Settings //
    setPoint += 0.01;
  }

  public void coralArticulateDown() {
    // Conditions //
    if (pivotRLimitSwitch.isPressed()) return;
    // Settings //
    setPoint -= 0.01;
  }
}
