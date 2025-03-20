// credit to: https://gist.github.com/Philanatidae/dfdad7761384808331f4fc42bbfbccb0
// helped with distance encoding
// credit to: https://docs.revrobotics.com/revlib/24-to-25
// migrating to 2025

// Directory //
package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

// Subsystem //
public class ElevatorSubsystem extends SubsystemBase {
  // Motor Data //
  TalonFX liftMotor;
  DutyCycleOut dutyCycleMotorRequest = new DutyCycleOut(0.0);
  PositionVoltage positionMotorRequest = new PositionVoltage(0).withSlot(0);
  double setPoint = Elevator.Positions.floorLevel;
  DigitalInput bottomLimitSwitch;
  // DigitalInput topLimitSwitch;

  boolean zeroing = true;

  double invertedCoefficient = 1;

  double manualDifference = Elevator.manualSpeed;

  public ElevatorSubsystem() {
    // Motor //
    liftMotor = new TalonFX(deviceIDs.elevatorID, "rio");
    // Configurations //
    TalonFXConfiguration configs = new TalonFXConfiguration();
    var slot0Configs = configs.Slot0;

    slot0Configs
        // .withKS(0.25) // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        // .withKV(0.12) // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V
        .withKP(Elevator.PID.P)
        .withKI(Elevator.PID.I)
        .withKD(Elevator.PID.D)
        .withStaticFeedforwardSign(
            StaticFeedforwardSignValue.UseClosedLoopSign) // TODO: CHECK THIS BEFORE RUNNING!
        .withGravityType(GravityTypeValue.Elevator_Static); // TODO: CHECK THIS BEFORE RUNNING!

    liftMotor.getConfigurator().apply(slot0Configs);
    // config = new SparkMaxConfig();
    // config.inverted(Elevator.MotorConfig.inverted).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    // liftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // encoder = new Encoder(0, 1);
    // encoder.setDistancePerPulse(1);
    // pidController = new PIDController(Elevator.PID.P, Elevator.PID.I, Elevator.PID.D);
    // feedforward = new ElevatorFeedforward(0, 0.1, 0);
    // pidController.setTolerance(200);
    bottomLimitSwitch = new DigitalInput(Elevator.IDs.bottomLimitSwitchID);
    // topLimitSwitch = new DigitalInput(Elevator.IDs.topLimitSwitchID);
  }

  public void run() {
    if (bottomLimitSwitchPressed()) { // When Done Zeroing //
      setPositionToZero();
      stopMotor();
      zeroing = false;
    }

    if (zeroing) { // Currently Zeroing //
      liftMotor.setControl(
          dutyCycleMotorRequest.withOutput(
              -0.4 * invertedCoefficient)); // TODO: CHECK IF INVERTED COEFFICIENT IS NEEDED!
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
      // Movement Profile //
      TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(80, 160));
      // Final target of 200 rot, 0 rps
      TrapezoidProfile.State m_goal =
          new TrapezoidProfile.State(
              setPoint, 0); // We want to be still after reaching the set point
      TrapezoidProfile.State output = new TrapezoidProfile.State(); // Calculation can't be a double

      // calculate the next profile setpoint
      output = m_profile.calculate(0.020, output, m_goal); // TODO: Tune T VALUE

      // create a position closed-loop request, voltage output, slot 0 configs
      final PositionVoltage m_request =
          new PositionVoltage(0)
              .withSlot(0)
              .withPosition(setPoint)
              .withVelocity(
                  MathUtil.clamp(output.velocity, -Elevator.elevatorSpeed, Elevator.elevatorSpeed));

      liftMotor.setControl(m_request);
    }
  }

  public void setPositionToZero() {
    liftMotor.setPosition(0);
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

  public double getCurrent() {
    return liftMotor
        .getStatorCurrent()
        .getValueAsDouble(); // TODO: CHECK IF THIS IS MOTOR CURRENT //
  }

  public double getPosition() {
    return liftMotor.getPosition().getValueAsDouble();
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
    return getPosition() == setPoint; // TODO: CHECK THIS //
  }
}
