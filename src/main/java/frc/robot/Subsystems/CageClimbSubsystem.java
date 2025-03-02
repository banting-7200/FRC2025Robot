package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

// Subsystem //
public class CageClimbSubsystem extends SubsystemBase {
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
  double POVtest;

  // Constructor //
  public CageClimbSubsystem() {
    climbMotor = new TalonSRX(deviceIDs.climberID);
  }

  public void checkPOVAndMove(double POVtest) {
    if (POVtest == 0) {
      climbMotor.set(TalonSRXControlMode.PercentOutput, Climber.Upspeed);
    } else if (POVtest == 180) {
      climbMotor.set(TalonSRXControlMode.PercentOutput, Climber.downSpeed);
    } else {
      climbMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
  }

  public void climbUp() {
    climbMotor.set(TalonSRXControlMode.PercentOutput, Climber.Upspeed);
  }

  public void climbDown() {
    climbMotor.set(TalonSRXControlMode.PercentOutput, Climber.downSpeed);
  }

  public void stopClimb() {
    climbMotor.set(TalonSRXControlMode.PercentOutput, 0);
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
