// Directory //
package frc.robot.Subsystems;

// Imports //
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralSystem;
import frc.robot.Constants.deviceIDs;

// Subsystem //
public class CoralIntakeSubsystem extends SubsystemBase {
  SparkMax pivotMotor;
  SparkMax intakeMotor;
  SparkClosedLoopController pidController;
  SparkMaxConfig config;
  // Limit Switches //
  SparkLimitSwitch pivotFLimitSwitch;
  SparkLimitSwitch pivotRLimitSwitch;
  SparkLimitSwitch intakeRLimitSwitch;

  public double setpoint = CoralSystem.Positions.carry;

  public CoralIntakeSubsystem() {
    pivotMotor = new SparkMax(deviceIDs.coralPivotID, MotorType.kBrushless);
    intakeMotor = new SparkMax(deviceIDs.coralIntakeID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    pidController = pivotMotor.getClosedLoopController();
    config.inverted(false).idleMode(IdleMode.kBrake);
    config.absoluteEncoder.positionConversionFactor(360).velocityConversionFactor(1);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(CoralSystem.PID.P, CoralSystem.PID.I, CoralSystem.PID.D);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotFLimitSwitch = pivotMotor.getForwardLimitSwitch();
    pivotRLimitSwitch = pivotMotor.getReverseLimitSwitch();

    intakeRLimitSwitch = intakeMotor.getForwardLimitSwitch();
  }

  public void run() {
    pidController.setReference(setpoint, ControlType.kPosition);
  }

  public boolean hasCoral() {
    return intakeRLimitSwitch.isPressed();
  }

  // Arm Motor

  public void moveToPosition(double setpoint) {
    this.setpoint = setpoint;
  }

  public double getPosition() {
    return pivotMotor.getAbsoluteEncoder().getPosition();
  }

  public boolean hasReachedSetpoint() {
    return Math.abs(getPosition() - setpoint) <= CoralSystem.Positions.safetyRange;
  }

  public void moveToIntakePosition() {
    moveToPosition(CoralSystem.Positions.intake);
  }

  public void moveToCarryPosition() {
    moveToPosition(CoralSystem.Positions.carry);
  }

  public void moveToOutputPosition() {
    moveToPosition(CoralSystem.Positions.dropOff);
  }

  // Intake Motor

  private void spinIntake(double speed) {

    intakeMotor.setVoltage(speed * 13);
  }

  public void intake() {
    spinIntake(CoralSystem.MotorSpeeds.intakeSpeed);
  }

  public void output() {
    spinIntake(CoralSystem.MotorSpeeds.outputSpeed);
  }

  public void stop() {
    intakeMotor.stopMotor();
  }
}
