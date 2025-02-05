// Directory //
package frc.robot.Subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
// Imports //
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.Coral.*;

// Subsystem //
public class CoralIntakeSubsystem {
  // Motors //
  SparkMax pivotMotor;
  SparkMax intakeMotor;
  // Motor Data //
  SparkAbsoluteEncoder encoder;
  SparkClosedLoopController pidController;
  SparkMaxConfig config;
  // Data //
  private double setPoint = Positions.carry;
  // Constructor //
  public CoralIntakeSubsystem() {
    // Init Motors //
    pivotMotor = new SparkMax(DeviceIDs.pivotMotor, null);
    intakeMotor = new SparkMax(DeviceIDs.intakeMotor, null);
    // Init Config //
    config = new SparkMaxConfig();
    // Init Encoder //
    encoder = pivotMotor.getAbsoluteEncoder();
    // Init PID //
    pidController = pivotMotor.getClosedLoopController();
    // Setup Config //
    config.inverted(true).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(PID.p, PID.i, PID.d);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    pidController.setReference(setPoint, ControlType.kPosition);
  }

  // #endregion
  // #region Level Methods //
  public void moveToLevelOne() {
    moveToPosition(Positions.levelOne);
  }

  public void moveToLevelTwo() {
    moveToPosition(Positions.levelTwo);
  }

  public void moveToLevelThree() {
    moveToPosition(Positions.levelThree);
  }

  public void moveToLevelFour() {
    moveToPosition(Positions.levelFour);
  }

  public void moveToIntake() {
    moveToPosition(Positions.intake);
  }

  public void moveToCarry() {
    moveToPosition(Positions.carry);
  }

  // #endregion
  // #region Action Methods //
  public void intake() {
    intakeMotor.set(1);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void output() {
    intakeMotor.set(-1);
  }

  // #endregion
}
