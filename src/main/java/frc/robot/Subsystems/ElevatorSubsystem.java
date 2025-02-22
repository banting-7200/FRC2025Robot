// credit to: https://gist.github.com/Philanatidae/dfdad7761384808331f4fc42bbfbccb0
// helped with distance encoding
// credit to: https://docs.revrobotics.com/revlib/24-to-25
// migrating to 2025

// Directory //
package frc.robot.Subsystems;

// Imports //
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.DriveBase;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Elevator.*;

// Subsystem //
public class ElevatorSubsystem {
  // Singleton //
  private static ElevatorSubsystem instance;
  // Instance Data //

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
    instance = this;
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
  public static ElevatorSubsystem getInstance() {
    return instance == null ? new ElevatorSubsystem() : instance;
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
    // Checks //
    if (position > Elevator.Positions.coralTwo)
      DriveBase.TranslationPID.p = DriveBase.TranslationPID.slowP;
    else DriveBase.TranslationPID.p = DriveBase.TranslationPID.normalP;
    // Settings //
    setPoint = position;
  }

  // #endregion //

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

  // #endregion
  // #region Algae Methods //
  public void moveToAlgaeOne() {
    moveToPosition(Positions.algaeOne);
  }

  public void moveToAlgaeTwo() {
    moveToPosition(Positions.algaeTwo);
  }

  // #endregion
  // #region Level Methods //
  public void moveToFloor() {
    moveToPosition(Positions.floorLevel);
    setPoint = 0;
  }

  public void moveToLevelOne() {
    if (Elevator.mode) // Algae //
    {
      moveToAlgaeOne();
    } else // Coral //
    {
      moveToCoralOne();
    }
  }

  public void moveToLevelTwo() {
    if (Elevator.mode) // Algae //
    {
      moveToAlgaeTwo();
    } else // Coral //
    {
      moveToCoralTwo();
    }
  }

  public void moveToLevelThree() {
    if (!Elevator.mode) // Coral //
    {
      moveToCoralThree();
    }
  }

  public void moveToLevelFour() {
    if (!Elevator.mode) // Coral //
    {
      moveToCoralFour();
    }
  }

  // #endregion //
  // #region Action Methods //
  public boolean switchMode() {
    return Elevator.mode = !Elevator.mode;
  }

  public void run() {
    pidController.setReference(setPoint, ControlType.kPosition);
  }
  // #endregion
}
