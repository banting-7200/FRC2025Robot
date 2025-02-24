// Package //
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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Algae.*;

// Subsystem //
public class AlgaeIntakeSubsystem {
  // Singleton //
  private static AlgaeIntakeSubsystem instance;
  // Subsystems //
  public CoralIntakeSubsystem coralArm;
  // Instance Data //
  public double setPoint;
  public boolean isUp;
  // Motors //
  public SparkMax pivotMotor;
  public SparkMax intakeMotor;
  // Motor Config //
  public SparkMaxConfig pivotConfig = new SparkMaxConfig();
  // Pivot Components //
  public SparkAbsoluteEncoder pivotEncoder;
  public SparkClosedLoopController pidController;
  // Limit Switches //
  public SparkLimitSwitch pivotFLimitSwitch;
  public SparkLimitSwitch pivotRLimitSwitch;
  public SparkLimitSwitch intakeRLimitSwitch;

  // Constructor //
  private AlgaeIntakeSubsystem() {
    // Init Motors //
    pivotMotor = new SparkMax(DeviceIDs.pivotMotor, MotorType.kBrushless);
    intakeMotor = new SparkMax(DeviceIDs.intakeMotor, MotorType.kBrushless);
    // Init Pivot Components //
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pidController = pivotMotor.getClosedLoopController();
    // Configure Pivot Motor //
    pivotConfig.inverted(true).idleMode(IdleMode.kBrake);
    pivotConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(PID.p, PID.i, PID.d);

    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Limit Switches //
    pivotFLimitSwitch = pivotMotor.getForwardLimitSwitch();
    pivotRLimitSwitch = pivotMotor.getReverseLimitSwitch();
    intakeRLimitSwitch = intakeMotor.getReverseLimitSwitch();
    // Update Singleton //
    coralArm = CoralIntakeSubsystem.getInstance();
    // Update Singleton //
    instance = this;
  }

  public static synchronized AlgaeIntakeSubsystem getInstance() {
    if (instance == null) return new AlgaeIntakeSubsystem();
    return instance;
  }

  // #region Base Methods //

  /**
   * Checks whether the Algae arm is low enough to move the coral arm
   *
   * @return Whether it's safe to move the coral arm
   */
  public boolean IsDown() {
    // Calculate //
    boolean isInPlace =
        pivotEncoder.getPosition() >= Positions.armDown - Positions.safetyRange
            && pivotEncoder.getPosition() <= Positions.armDown + Positions.safetyRange;
    // Return //
    return isInPlace;
  }

  /**
   * Sets the target angle of the motor
   *
   * @param setPoint The angle
   */
  public void moveToPosition(double setPoint) {
    this.setPoint = setPoint;
  }

  /**
   * Gets the target angle of the motor
   *
   * @return the current angle of the motor
   */
  public double getSetPoint() {
    return setPoint;
  }

  /**
   * Gets the precise angle of the motor right now
   *
   * @return The angle of the motor
   */
  public double getPosition() {
    return pivotEncoder.getPosition();
  }

  /** Update method */
  public void run() {
    // Settings //
    if (pivotEncoder.getPosition() > Positions.armUp + 2
        && pivotEncoder.getPosition() < Positions.armDown - 2) {
      pidController.setReference(setPoint, ControlType.kPosition);
    } else {
      pivotMotor.stopMotor();
    }
  }

  public boolean hasAlgae() {
    return intakeRLimitSwitch.isPressed();
  }

  // #endregion
  // #region Action Methods //
  /** Intake Algae */
  public void Intake() {
    // Begin Intake //
    intakeMotor.set(-MotorSpeeds.intakeSpeed);
    // Wait Till Successful Intake //
    new WaitUntilCommand(intakeRLimitSwitch::isPressed)
        .andThen(
            () -> {
              StopIntake();
            });
  }

  /** Stop the intake motor */
  public void StopIntake() {
    intakeMotor.stopMotor();
  }

  /** Spit out algae */
  public void Output() {
    intakeMotor.set(MotorSpeeds.outputSpeed);
  }

  // Arm Controls //
  /** Moves the arm up */
  public void armUp() {
    new WaitUntilCommand(coralArm::isAtCarryPosition)
        .unless(this::hasAlgae)
        .andThen(
            () -> { // Move The Algae Arm to Carry Position //
              moveToPosition(Positions.armUp);
              // Update Values //
              isUp = true;
            });
  }

  /** Moves the arm down */
  public void armDown() {
    moveToPosition(Positions.armDown);
    isUp = false;
  }

  /** Toggles between arm up and arm down */
  public void armToggle() {
    // Checks //
    if (isUp) armDown();
    else armUp();
  }

  /** Checks whether Coral Arm is ready for Algae Arm to move up, if not then get coral arm ready */
  public void checkCoralArm() {
    // Data //
    boolean isCoralArmReady = coralArm.isAtCarryPosition();
    // Conditions //
    if (isCoralArmReady) return;
    // Move To Carrying Position //
    coralArm.moveToCarry();
  }

  // #endregion //
  public void algaeArmArticulateUp() { // Up is positive + //
    // Conditions //
    if (pivotFLimitSwitch.isPressed()) return;
    // Settings //
    setPoint += 0.01;
  }

  public void algaeArmArticulateDown() { // Down is negative - //

    // Conditions //
    if (pivotRLimitSwitch.isPressed()) return;
    // Settings //
    setPoint -= 0.01;
  }

  public void algaeArmArticulateOff() {
    pivotMotor.stopMotor();
  }
}
