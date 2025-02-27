// credit to: https://gist.github.com/Philanatidae/dfdad7761384808331f4fc42bbfbccb0
// helped with distance encoding
// credit to: https://docs.revrobotics.com/revlib/24-to-25
// migrating to 2025

// Directory //
package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

// Subsystem //
public class ElevatorSubsystem extends SubsystemBase {
  SparkMax liftMotor;
  SparkMaxConfig config;
  double setPoint;
  Encoder encoder;
  PIDController pidController;
  DigitalInput bottomLimitSwitch;
  DigitalInput topLimitSwitch;

  boolean zeroing = true;

  public ElevatorSubsystem() {
    liftMotor = new SparkMax(deviceIDs.elevatorID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config.inverted(Elevator.MotorConfig.inverted).idleMode(IdleMode.kBrake);
    liftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder = new Encoder(0, 1);
    encoder.setDistancePerPulse(1);
    pidController = new PIDController(Elevator.PID.P, Elevator.PID.I, Elevator.PID.D);
    pidController.setTolerance(5, 10);
    bottomLimitSwitch = new DigitalInput(Elevator.IDs.bottomLimitSwitchID);
    topLimitSwitch = new DigitalInput(Elevator.IDs.topLimitSwitchID);
  }

  public void run() {
    if (bottomLimitSwitchPressed()) {
      System.out.println("zeroing");
      encoder.reset();
    }
    // if (setPoint > getPosition() && !belowUpperSoftLimits()) return;
    // if (setPoint < getPosition() && !aboveLowerSoftLimits() && !zeroing) return;
    // liftMotor.set(pidController.calculate(encoder.getDistance(), setPoint));
  }

  public void zero() {
    zeroing = true;
    liftMotor.set(0.1);
  }

  public boolean belowUpperSoftLimits() {
    return getPosition() < Elevator.Positions.top - Elevator.Positions.safeZone;
  }

  public boolean aboveLowerSoftLimits() {
    return getPosition() > Elevator.Positions.safeZone;
  }

  public void stopMotor() {
    liftMotor.stopMotor();
  }

  public boolean bottomLimitSwitchPressed() {
    return !bottomLimitSwitch.get();
  }

  public boolean topLimitSwitchPressed() {
    return !topLimitSwitch.get();
  }

  public void setMotorSpeed(double speed) {
    if (topLimitSwitchPressed() && speed > 0) {
      liftMotor.set(0);
      return;
    } else if (bottomLimitSwitchPressed() && speed < 0) {
      liftMotor.set(0);
      return;
    }
    liftMotor.set(speed);
  }

  public void moveToPosition(double setPoint) {
    zeroing = false;
    this.setPoint = setPoint;
  }

  public boolean hasReachedSetpoint() {
    // return pidController.atSetpoint();
    return true;
  }

  public double getPosition() {
    return encoder.getDistance();
  }
}
