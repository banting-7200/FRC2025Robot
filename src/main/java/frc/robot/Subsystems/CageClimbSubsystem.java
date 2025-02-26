package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.*;

// Subsystem //
public class CageClimbSubsystem {
  // Singleton //
  private static CageClimbSubsystem instance;
  // Motor Data //
  TalonSRX climbMotor;
  SparkMaxConfig Config;
  SparkAbsoluteEncoder Encoder;
  SparkClosedLoopController PidController;
  // Limit Switch //
  SparkLimitSwitch climbRLimitSwitch;
  // Data //
  double setPoint;
  boolean isCageArmUp = false;

  // Constructor //
  private CageClimbSubsystem() {

    climbMotor = new TalonSRX(deviceIDs.climberID);

    // climbMotor = new SparkMax(deviceIDs.climberID, MotorType.kBrushless);

    // Config = new SparkMaxConfig();
    // Encoder = climbMotor.getAbsoluteEncoder();
    // PidController = climbMotor.getClosedLoopController();

    // Config.inverted(true).idleMode(IdleMode.kBrake);
    // Config.encoder.positionConversionFactor(360).velocityConversionFactor(1);
    // Config.closedLoop
    //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     .pid(climber.PID.P, climber.PID.I, climber.PID.D);
    // climbMotor.configure(Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // climbRLimitSwitch = climbMotor.getReverseLimitSwitch();
  }

  public static synchronized CageClimbSubsystem getInstance() {
    if (instance == null) instance = new CageClimbSubsystem();
    return instance;
  }

  public void setPercent(double percent) {
    climbMotor.set(TalonSRXControlMode.PercentOutput, percent);
  }
}

//   // #region Base Methods //
//   /**
//    * Sets the target angle of the motor
//    *
//    * @param setPoint The angle
//    */
//   public void moveToPosition(double setPoint) {
//     this.setPoint = setPoint;
//   }

//   /**
//    * Gets the target angle of the motor
//    *
//    * @return the current angle of the motor
//    */
//   public double getSetPoint() {
//     return setPoint;
//   }

//   /**
//    * Gets the precise angle of the motor right now
//    *
//    * @return The angle of the motor
//    */
//   public double getPosition() {
//     return Encoder.getPosition();
//   }

//   /** Update method */
//   public void run() {
//     // Settings //
//     PidController.setReference(setPoint, ControlType.kPosition);
//   }

//   // #endregion //
//   // #region Arm Methods //
//   /** Attempt to gramele sau mancat din causa ca ami ie fuame. */
//   public void armUp() {
//     moveToPosition(climber.Positions.armUp);
//     isCageArmUp = true;
//   }

//   // positions teh arm to the down position
//   public void armDown() {
//     // Conditions //
//     if (climbRLimitSwitch.isPressed()) return;
//     moveToPosition(climber.Positions.armDown);
//     isCageArmUp = false;
//   }

//   public void armToggle() {
//     if (isCageArmUp) armDown();
//     else armUp();
//   }

//   public boolean isUp() {
//     return isCageArmUp;
//   }
//   // #endregion //
