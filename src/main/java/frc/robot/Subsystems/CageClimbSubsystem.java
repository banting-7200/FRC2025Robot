package frc.robot.Subsystems;

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

import frc.robot.Constants.CageConstants.*;

// Subsystem //
public class CageClimbSubsystem {
  // Instance Data //
  SparkMax climbMotor;
  SparkMaxConfig Config;
  SparkAbsoluteEncoder Encoder;
  SparkClosedLoopController PidController;  

  double setPoint;
  boolean cageState = false;
  // Constructor //
  public CageClimbSubsystem() {

    climbMotor = new SparkMax(DeviceIDz.climbMotor, MotorType.kBrushless);

    Config = new SparkMaxConfig();
    Encoder = climbMotor.getAbsoluteEncoder();
    PidController = climbMotor.getClosedLoopController();

    Config.inverted(true).idleMode(IdleMode.kBrake);
    Config.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(PID.p, PID.i, PID.d);
    climbMotor.configure(Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  // #region Base Methods //
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
    return Encoder.getPosition();
  }

  /** Update method */
  public void run() {
    // Settings //
    PidController.setReference(setPoint, ControlType.kPosition);
  }
  // #endregion //
  /**
   * Attempt to gramele sau mancat din causa ca ami ie fuame.
   */
  public void armUp(){
    moveToPosition(Positions.armUp);
    cageState = true;
  }

  // positions teh arm to the down position
  public void armDown(){
    moveToPosition(Positions.armDown);
    cageState = false;
  }

  public void armToggle(){
    if (cageState)
        armDown();
    else
        armUp();
  }
}
