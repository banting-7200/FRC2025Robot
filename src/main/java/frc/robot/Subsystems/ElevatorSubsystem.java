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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

// Subsystem //
public class ElevatorSubsystem extends SubsystemBase {
  SparkMax liftMotor;
  SparkMaxConfig config;
  double setPoint = Elevator.Positions.floorLevel;
  Encoder encoder;
  PIDController pidController;
  ElevatorFeedforward feedforward;
  DigitalInput bottomLimitSwitch;
  // DigitalInput topLimitSwitch;

  boolean zeroing = true;

  double invertedCoefficient = 1;

  public ElevatorSubsystem() {
    liftMotor = new SparkMax(deviceIDs.elevatorID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config.inverted(Elevator.MotorConfig.inverted).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    liftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder = new Encoder(0, 1);
    encoder.setDistancePerPulse(1);
    pidController = new PIDController(Elevator.PID.P, Elevator.PID.I, Elevator.PID.D);
    feedforward = new ElevatorFeedforward(0, 0.1, 0);
    pidController.setTolerance(200);
    bottomLimitSwitch = new DigitalInput(Elevator.IDs.bottomLimitSwitchID);
    // topLimitSwitch = new DigitalInput(Elevator.IDs.topLimitSwitchID);
  }

  public void run() {
    System.out.println("Current Position: " + getPosition());
    System.out.println("Current: " + getCurrent());
    if (bottomLimitSwitchPressed()) {
      zeroing = false;
      encoder.reset();
      setMotorSpeed(0);
    }
    if (zeroing) {
      System.out.println("zeroing");
      setMotorSpeed(Elevator.reZeroSpeed * invertedCoefficient);
    } else {
      if (setPoint < getPosition() && (!belowUpperSoftLimits())) {
        setMotorSpeed(0);
        System.out.println("Above Upper Limits");
        return;
      }
      if (setPoint > getPosition() && (!aboveLowerSoftLimits() || bottomLimitSwitchPressed())) {
        setMotorSpeed(0);
        System.out.println("Below Lower Limits");
        return;
      }
      double output = pidController.calculate(encoder.getDistance(), setPoint);

      if (getPosition() > Elevator.Positions.algaeOne) {
        output = MathUtil.clamp(output, -1, 0.5);
      }
      output *= (invertedCoefficient);
      liftMotor.set(output);
      System.out.println("Moving");
    }
  }

  public void zero() {
    zeroing = true;
  }

  public boolean belowUpperSoftLimits() {
    return getPosition() > Elevator.Positions.top + Elevator.Positions.safeZone;
  }

  public boolean aboveLowerSoftLimits() {
    return getPosition() < -Elevator.Positions.safeZone;
  }

  public void stopMotor() {
    liftMotor.stopMotor();
  }

  public void setMotorSpeed(double speed) {
    liftMotor.set(speed);
  }

  public boolean bottomLimitSwitchPressed() {
    return !bottomLimitSwitch.get();
  }

  //   public boolean topLimitSwitchPressed() {
  //     return !topLimitSwitch.get();
  //   }

  public void moveToPosition(double setPoint) {
    zeroing = false;
    this.setPoint = setPoint;
  }

  public boolean hasReachedSetpoint() {
    return pidController.atSetpoint();
  }

  public double getPosition() {
    return encoder.getDistance();
  }

  public double getCurrent() {
    return liftMotor.getOutputCurrent();
  }

  public void flipMotor() {
    invertedCoefficient *= -1;
  }
}
