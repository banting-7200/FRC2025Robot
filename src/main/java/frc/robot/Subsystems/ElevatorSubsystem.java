// credit to: https://gist.github.com/Philanatidae/dfdad7761384808331f4fc42bbfbccb0
// helped with distance encoding
// credit to: https://docs.revrobotics.com/revlib/24-to-25
// migrating to 2025

// Directory //
package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

// Subsystem //
public class ElevatorSubsystem extends SubsystemBase {
  // Motor Data //
  TalonFX liftMotor;
  DutyCycleOut dutyCycleMotorRequest = new DutyCycleOut(0.0);
  Encoder encoder;
  PIDController pidController;
  // Motor Position //
  double setPoint = Elevator.Positions.floorLevel;
  DigitalInput bottomLimitSwitch;
  // DigitalInput topLimitSwitch;
  boolean zeroing = true; // Zero robot when it starts //
  double invertedCoefficient = 1;

  double manualDifference = Elevator.manualSpeed;

  public ElevatorSubsystem() {
    // Motor //
    liftMotor = new TalonFX(deviceIDs.elevatorID, "rio");
    // Configurations //
    TalonFXConfiguration configs = new TalonFXConfiguration();
    // Motor Configurations //
    MotorOutputConfigs motorConfigs = configs.MotorOutput;

    motorConfigs
        .withInverted(Constants.Elevator.MotorConfig.inverted)
        .withNeutralMode(NeutralModeValue.Brake);

    liftMotor.getConfigurator().apply(motorConfigs);
    // Encoder Setup //
    encoder = new Encoder(0, 1);
    encoder.setDistancePerPulse(1);
    // PID Setup //
    pidController = new PIDController(Elevator.PID.P, Elevator.PID.I, Elevator.PID.D);
    pidController.setTolerance(150);
    // Limit Switch Setup //
    bottomLimitSwitch = new DigitalInput(Elevator.IDs.bottomLimitSwitchID);
    // topLimitSwitch = new DigitalInput(Elevator.IDs.topLimitSwitchID);
  }

  public void run() {
    System.out.println("Elevator Position: " + getPosition());
    // When Limit Switch is hit //
    if (bottomLimitSwitchPressed()) { // When Done Zeroing //
      // Zero && Stop Motor //
      setPositionToZero();
      stopMotor();
      // Stop Zeroing //
      zeroing = false;
    }

    if (zeroing) { // Currently Zeroing //
      liftMotor.set(0.4);
    } else // Not Zeroing //
    {
      // If Above Upper Soft Limits //
      if (setPoint < getPosition() && (!belowUpperSoftLimits())) {
        stopMotor();
        return;
      }

      // If Below Lower Soft Limits //
      if (setPoint > getPosition() && (!aboveLowerSoftLimits() || bottomLimitSwitchPressed())) {
        stopMotor();
        return;
      }
      double output =
          pidController.calculate(encoder.getDistance(), setPoint); // Calculation can't be a double

      if (getPosition() > Elevator.Positions.algaeOne / 4) // If Below First Level //
      output = MathUtil.clamp(output, -.75, .25);
      else output = MathUtil.clamp(output, -1, .75);

      // create a position closed-loop request, voltage output, slot 0 configs
      liftMotor.set(output);
    }
  }

  public void setPositionToZero() {
    encoder.reset();
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
    // Cancel Zeroing //
    zeroing = false;
    // Set Goal //
    this.setPoint = setPoint;
  }

  public double getCurrent() {
    return liftMotor
        .getStatorCurrent()
        .getValueAsDouble(); // TODO: CHECK IF THIS IS MOTOR CURRENT //
  }

  public double getPosition() {
    return encoder.getDistance();
  }

  public double getSetpoint() {
    return setPoint;
  }

  public void flipMotor() {
    invertedCoefficient *= -1;
  }

  public void moveUp() {
    setPoint -= manualDifference;
  }

  public void moveDown() {
    setPoint += manualDifference;
  }

  public boolean hasReachedSetpoint() {
    return pidController.atSetpoint();
  }
}
