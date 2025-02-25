// Package //
package frc.robot.Subsystems;

// Imports //
import com.revrobotics.spark.SparkAbsoluteEncoder;
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
import frc.robot.Constants.*;

// Subsystem //
public class AlgaeIntakeSubsystem extends SubsystemBase {
  public double setpoint;
  public boolean isArmUp;
  public SparkMax pivotMotor;
  public SparkMax intakeMotor;
  public SparkMaxConfig pivotConfig = new SparkMaxConfig();
  public SparkAbsoluteEncoder pivotEncoder;
  public SparkClosedLoopController pidController;

  public SparkLimitSwitch pivotFLimitSwitch;
  public SparkLimitSwitch pivotRLimitSwitch;
  public SparkLimitSwitch intakeRLimitSwitch;

  public boolean hasAlgae;

  public AlgaeIntakeSubsystem() {

    pivotMotor = new SparkMax(deviceIDs.algaePivotID, MotorType.kBrushless);
    intakeMotor = new SparkMax(deviceIDs.algaeIntakeID, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pidController = pivotMotor.getClosedLoopController();
    pivotConfig.inverted(true).idleMode(IdleMode.kBrake);
    pivotConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(algaeSystem.PID.P, algaeSystem.PID.I, algaeSystem.PID.D);

    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotFLimitSwitch = pivotMotor.getForwardLimitSwitch();
    pivotRLimitSwitch = pivotMotor.getReverseLimitSwitch();
    intakeRLimitSwitch = intakeMotor.getReverseLimitSwitch();
  }

  public boolean hasReachedSetpoint() {
    double acceptableRange = 5;
    return Math.abs(pivotEncoder.getPosition() - setpoint) <= acceptableRange;
  }

  public void moveToPosition(double setpoint) {
    this.setpoint = setpoint;
  }

  public void run() {
    pidController.setReference(setpoint, ControlType.kPosition);
  }

  public boolean hasAlgae() {
    hasAlgae = intakeRLimitSwitch.isPressed();
    return hasAlgae;
  }

  public void spinIntake(double speed) {
    intakeMotor.set(speed);
  }

  /** Stop the intake motor */
  public void StopIntake() {
    intakeMotor.stopMotor();
  }

  public void shutOffPivotMotor() {
    pivotMotor.stopMotor();
  }

  public void shutOffIntakeMotor() {
    intakeMotor.stopMotor();
  }
}
