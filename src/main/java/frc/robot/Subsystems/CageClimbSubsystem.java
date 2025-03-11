package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class CageClimbSubsystem {
  TalonFX falcon500;
  DutyCycleOut dutyCycleMotorRequest = new DutyCycleOut(0.0);
  PositionVoltage positionMotorRequest = new PositionVoltage(0).withSlot(0);
  DigitalInput bottomLimitSwitch = new DigitalInput(9);
  double currentPosition;
  double setpoint = 100;
  boolean setpointState = true;
  boolean doesCodeHaveMotorPriority = true;
  public boolean hasBeenZeroed = false;

  public CageClimbSubsystem() {
    falcon500 = new TalonFX(7, "rio");
    TalonFXConfiguration configs = new TalonFXConfiguration();
    var slot0Configs = configs.Slot0;
    slot0Configs.kP = 0.3;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.005;
    // headMotor.getConfigurator().apply(configs);
    falcon500.getConfigurator().apply(slot0Configs);
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  public void increaseSetpoint() {
    setpoint += 2;
    if (setpoint > 350) setpoint = 350;
  }

  public void decreaseSetpoint() {
    setpoint -= 1;
    if (setpoint < 0) setpoint = 0;
  }

  //   public void toggleSetpoint() {
  //     setpointState = !setpointState;
  //     if (setpointState) setpoint = 350;
  //     if (!setpointState) setpoint = 0;
  //   }

  public void autoZero() {
    if (!hasBeenZeroed) {
      doesCodeHaveMotorPriority = true;
      if (!getBottomLimitSwitch()) {
        falcon500.setControl(dutyCycleMotorRequest.withOutput(-0.4));
      } else {
        falcon500.setControl(dutyCycleMotorRequest.withOutput(0));
        setPositionToZero();
        setpoint = 100;
        falcon500.setControl(positionMotorRequest.withPosition(setpoint).withSlot(0));
        doesCodeHaveMotorPriority = false;
        hasBeenZeroed = true;
      }
    }
  }

  public double getPosition() {
    currentPosition = falcon500.getPosition().getValueAsDouble();
    return currentPosition;
  }

  public void setPosition(double setpoint) {
    falcon500.setPosition(setpoint);
  }

  public void setPositionToZero() {
    falcon500.setPosition(0);
  }

  public void run() {
    if (!doesCodeHaveMotorPriority) {
      System.out.println("moving");
      falcon500.setControl(positionMotorRequest.withPosition(setpoint).withSlot(0));
    }
    System.out.println(
        "Position = "
            + getPosition()
            + " | Setpoint = "
            + setpoint
            + " | Limit = "
            + getBottomLimitSwitch());
  }
}
