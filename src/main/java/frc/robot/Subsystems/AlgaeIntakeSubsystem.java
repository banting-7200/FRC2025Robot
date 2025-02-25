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
import frc.robot.Constants.AlgaeSystem;
import frc.robot.Constants.deviceIDs;

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

  public AlgaeIntakeSubsystem() {

    pivotMotor = new SparkMax(deviceIDs.algaePivotID, MotorType.kBrushless);
    intakeMotor = new SparkMax(deviceIDs.algaeIntakeID, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pidController = pivotMotor.getClosedLoopController();
    pivotConfig.inverted(true).idleMode(IdleMode.kBrake);
    pivotConfig.encoder.positionConversionFactor(360).velocityConversionFactor(1);
    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(AlgaeSystem.PID.P, AlgaeSystem.PID.I, AlgaeSystem.PID.D);

    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotFLimitSwitch = pivotMotor.getForwardLimitSwitch();
    pivotRLimitSwitch = pivotMotor.getReverseLimitSwitch();
    intakeRLimitSwitch = intakeMotor.getReverseLimitSwitch();
  }

  public void run() {
    pidController.setReference(setpoint, ControlType.kPosition);
  }

  public boolean hasAlgae() {
    return intakeRLimitSwitch.isPressed();
  }

  // Arm Motor

  public void moveToPosition(double setpoint) {
    this.setpoint = setpoint;
  }

  public double getPosition() {
    return pivotEncoder.getPosition();
  }

  public boolean hasReachedSetpoint() {
    return Math.abs(getPosition() - setpoint) <= AlgaeSystem.Positions.safetyRange;
  }

  public void moveToUpPosition() {
    moveToPosition(AlgaeSystem.Positions.up);
  }

  public void moveToShootPosition() {
    moveToPosition(AlgaeSystem.Positions.shoot);
  }

  public void moveToDownPosition() {
    moveToPosition(AlgaeSystem.Positions.down);
  }

  // Intake Motor

  private void spinIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void intake() {
    spinIntake(AlgaeSystem.MotorSpeeds.intakeSpeed);
  }

  public void output() {
    spinIntake(AlgaeSystem.MotorSpeeds.outputSpeed);
  }

  public void shoot() {
    spinIntake(AlgaeSystem.MotorSpeeds.shootSpeed);
  }

  public void stop() {
    intakeMotor.stopMotor();
  }
}
