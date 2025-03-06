package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.*;

public class CageClimbSubsystem {
  TalonFX falcon500;
  DutyCycleOut dutyCycleMotorRequest = new DutyCycleOut(0.0);
  PositionVoltage positionMotorRequest = new PositionVoltage(0).withSlot(0);
  DigitalInput bottomLimitSwitch = new DigitalInput(Climber.limitSwitchID);

  double currentPosition;
  double setpoint;

  boolean setpointState;
  boolean speedControl = false;
  public boolean doesCodeHaveMotorPriority = true;

  public CageClimbSubsystem() {
    falcon500 = new TalonFX(deviceIDs.climberID);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    var slot0Configs = configs.Slot0;
    // Set PID values //
    slot0Configs.kP = Climber.PID.P;
    slot0Configs.kI = Climber.PID.I;
    slot0Configs.kD = Climber.PID.D;
    // configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; TODO: FIX INVERSIONS
    falcon500.getConfigurator().apply(slot0Configs);
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  public void increaseSetpoint() {
    setpoint += Climber.Upspeed;
  }

  public void decreaseSetpoint() {
    setpoint -= Climber.downSpeed;
  }

  public void toggleSetpoint() {
    setpointState = !setpointState;
    if (setpointState) setpoint = Climber.Positions.armOut;
    if (!setpointState) setpoint = Climber.Positions.armIn;
  }

  public void autoZero() {
    if (doesCodeHaveMotorPriority) {
      if (!getBottomLimitSwitch()) {
        falcon500.setControl(dutyCycleMotorRequest.withOutput(-0.1));
      } else {
        falcon500.setControl(dutyCycleMotorRequest.withOutput(0));
        setPositionToZero();
        setpoint = Climber.Positions.armIn;
        falcon500.setControl(positionMotorRequest.withPosition(setpoint).withSlot(0));
        doesCodeHaveMotorPriority = false;
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
      falcon500.setControl(positionMotorRequest.withPosition(setpoint).withSlot(0));
    }
  }
}
