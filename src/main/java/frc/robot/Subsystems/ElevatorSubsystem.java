// credit to: https://gist.github.com/Philanatidae/dfdad7761384808331f4fc42bbfbccb0
// helped with distance encoding
// credit to: https://docs.revrobotics.com/revlib/24-to-25
// migrating to 2025

// Directory //
package frc.robot.Subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
// Imports //
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ElevatorConstants.*;

// Subsystem //
public class ElevatorSubsystem {
  // Static Data //
  private static ElevatorSubsystem lift;
  // Motors //
  SparkMax liftMotor;
  SparkMaxConfig config;
  // Encoder //
  SparkAbsoluteEncoder encoder;
  double setPoint;
  // PID //
  SparkClosedLoopController pidController;

  // Constructors //
  private ElevatorSubsystem() {
    // Init Static Data //
    lift = this;
    // Init Motor //
    liftMotor = new SparkMax(MotorConfig.canID, MotorType.kBrushless);
    config = new SparkMaxConfig();

    // Motor Configuration //
    config.inverted(MotorConfig.inverted).idleMode(IdleMode.kBrake);
    config
        .encoder
        .positionConversionFactor(MotorConfig.positionConversionFactor)
        .velocityConversionFactor(MotorConfig.velocityConversionFactor);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(PID.p, PID.i, PID.d);
    // Encoder //
    encoder = liftMotor.getAbsoluteEncoder();
    // PID //
    pidController = liftMotor.getClosedLoopController();
    // Configurations //
    liftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // #region Static Methods //
  /**
   * Gets the Subystem
   *
   * @return The Elevator Subsystem
   */
  public static ElevatorSubsystem get() {
    return lift == null ? new ElevatorSubsystem() : lift;
  }

  // #endregion
  // #region Base Methods //
  /**
   * Returns the height of the elevator
   *
   * @return The height of the elevator
   */
  public double getCurrentPosition() {
    return encoder.getPosition();
  }

  public double getSetPoint() {
    return setPoint;
  }

  /**
   * Moves the elevator to a specific position
   *
   * @param position The position to move to
   */
  private void moveToPosition(double position) {
    setPoint = position;
  }

  // #endregion
  // #region Coral Methods //
  public void moveToCoralOne() {
    moveToPosition(Positions.coralOne);
  }

  public void moveToCoralTwo() {
    moveToPosition(Positions.coralTwo);
  }

  public void moveToCoralThree() {
    moveToPosition(Positions.coralThree);
  }

  public void moveToCoralFour() {
    moveToPosition(Positions.coralFour);
  }

  public void moveToIntakeCoral() {
    moveToPosition(Positions.intake);
  }

  // #endregion
  // #region Algae Methods //
  public void moveToAlgaeOne() {
    moveToPosition(Positions.algaeOne);
  }

  public void moveToAlgaeTwo() {
    moveToPosition(Positions.algaeTwo);
  }

  public void moveToProcessor() {
    moveToPosition(Positions.processor);
  }

  public void moveToNet() {
    moveToPosition(Positions.net);
  }

  // #endregion
  // #region Action Methods //
  public void run() {
    pidController.setReference(setPoint, ControlType.kPosition);
  }
  // #endregion
}
