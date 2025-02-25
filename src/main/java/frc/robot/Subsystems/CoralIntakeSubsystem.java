// Directory //
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
public class CoralIntakeSubsystem extends SubsystemBase {
  SparkMax pivotMotor;
  SparkMax intakeMotor;
  SparkAbsoluteEncoder encoder;
  SparkClosedLoopController pidController;
  SparkMaxConfig config;
  // Limit Switches //
  SparkLimitSwitch pivotFLimitSwitch;
  SparkLimitSwitch pivotRLimitSwitch;
  SparkLimitSwitch intakeRLimitSwitch;
  public double setpoint = coralSystem.Positions.carry;
  public boolean hasCoral;

  public CoralIntakeSubsystem() {
    pivotMotor = new SparkMax(deviceIDs.coralPivotID, MotorType.kBrushless);
    intakeMotor = new SparkMax(deviceIDs.coralIntakeID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    encoder = pivotMotor.getAbsoluteEncoder();
    pidController = pivotMotor.getClosedLoopController();
    config.inverted(true).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(360).velocityConversionFactor(1);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(coralSystem.PID.P, coralSystem.PID.I, coralSystem.PID.D);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotFLimitSwitch = pivotMotor.getForwardLimitSwitch();
    pivotRLimitSwitch = pivotMotor.getReverseLimitSwitch();

    intakeRLimitSwitch = intakeMotor.getForwardLimitSwitch();
  }

  public void moveToPosition(double setpoint) {
    this.setpoint = setpoint;
  }

  public boolean hasReachedSetpoint() {
    double acceptableRange = 5;
    return Math.abs(encoder.getPosition() - setpoint) <= acceptableRange;
  }

  public void run() {
    pidController.setReference(setpoint, ControlType.kPosition);
  }

  public boolean hasCoral() {
    hasCoral = intakeRLimitSwitch.isPressed();
    return hasCoral;
  }

  public void spinIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }
}
