// credit to: https://gist.github.com/Philanatidae/dfdad7761384808331f4fc42bbfbccb0
// helped with distance encoding
// credit to: https://docs.revrobotics.com/revlib/24-to-25
// migrating to 2025

// Directory //
package frc.robot.Subsystems;

// Imports //
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

// Subsystem //
public class ElevatorSubsystem extends SubsystemBase {
  SparkMax liftMotor;
  SparkMaxConfig config;
  SparkAbsoluteEncoder encoder;
  double setPoint;
  SparkClosedLoopController pidController;

  public ElevatorSubsystem() {
    liftMotor = new SparkMax(deviceIDs.elevatorID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config.inverted(Elevator.MotorConfig.inverted).idleMode(IdleMode.kBrake);
    config
        .encoder
        .positionConversionFactor(Elevator.MotorConfig.positionConversionFactor)
        .velocityConversionFactor(Elevator.MotorConfig.velocityConversionFactor);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Elevator.PID.P, Elevator.PID.I, Elevator.PID.D);
    encoder = liftMotor.getAbsoluteEncoder();
    pidController = liftMotor.getClosedLoopController();
    liftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void moveToPosition(double setPoint) {
    this.setPoint = setPoint;
  }

  public void run() {
    pidController.setReference(setPoint, ControlType.kPosition);
  }

  public boolean hasReachedSetpoint() {
    return false;
  }

  public void zeroEncoder() {}
}
