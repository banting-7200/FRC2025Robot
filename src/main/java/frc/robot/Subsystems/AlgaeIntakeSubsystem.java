// Package //
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
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeSystem;
import frc.robot.Constants.deviceIDs;

// Subsystem //
public class AlgaeIntakeSubsystem extends SubsystemBase {
  public double setpoint = AlgaeSystem.Positions.up;
  public boolean isArmUp;
  public SparkMax pivotMotor;
  public SparkMax intakeMotor;
  public SparkMax intakeMotor2;
  public SparkMaxConfig pivotConfig = new SparkMaxConfig();
  public SparkMaxConfig intakeConfig = new SparkMaxConfig();
  public SparkMaxConfig intake2Config = new SparkMaxConfig();

  // public SparkAbsoluteEncoder pivotEncoder;
  public SparkClosedLoopController pidController;

  public SparkLimitSwitch pivotFLimitSwitch;
  public SparkLimitSwitch pivotRLimitSwitch;
  public SparkLimitSwitch intakeRLimitSwitch;

  public AlgaeIntakeSubsystem() {

    pivotMotor = new SparkMax(deviceIDs.algaePivotID, MotorType.kBrushless);
    intakeMotor = new SparkMax(deviceIDs.algaeIntakeID, MotorType.kBrushless);
    intakeMotor2 = new SparkMax(5, MotorType.kBrushless);
    // pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pidController = pivotMotor.getClosedLoopController();

    intakeConfig.inverted(false).idleMode(IdleMode.kBrake);

    pivotConfig.inverted(false).idleMode(IdleMode.kBrake);
    pivotConfig
        .absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(1)
        .inverted(true);
    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(AlgaeSystem.PID.P, AlgaeSystem.PID.I, AlgaeSystem.PID.D);

    pivotConfig
        .limitSwitch
        .forwardLimitSwitchEnabled(true)
        .forwardLimitSwitchType(Type.kNormallyClosed)
        .reverseLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen);

    intakeConfig
        .limitSwitch
        .reverseLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyClosed);

    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intake2Config.inverted(true).idleMode(IdleMode.kBrake).follow(intakeMotor, true);

    intakeMotor2.configure(
        intake2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotFLimitSwitch = pivotMotor.getForwardLimitSwitch();
    pivotRLimitSwitch = pivotMotor.getReverseLimitSwitch();
    intakeRLimitSwitch = intakeMotor.getReverseLimitSwitch();
  }

  public void run() {
    // System.out.println("trying to move to " + setpoint + " | Current position = " +
    // getPosition());
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
    return pivotMotor.getAbsoluteEncoder().getPosition();
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
