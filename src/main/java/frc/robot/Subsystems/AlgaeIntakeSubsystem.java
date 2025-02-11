// Package //
package frc.robot.Subsystems;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
// Imports //
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.Algae.*;
// Subsystem //
public class AlgaeIntakeSubsystem
{
    // Instance Data //
    private double setPoint;
    // Motors //
    private SparkMax pivotMotor;
    private SparkMax intakeMotor;
    // Motor Config //
    private SparkMaxConfig pivotConfig = new SparkMaxConfig();
    // Pivot Components //
    private SparkAbsoluteEncoder pivotEncoder;
    private SparkClosedLoopController pidController;
    // Constructor //
    public AlgaeIntakeSubsystem()
    {
        // Init Motors //
        pivotMotor = new SparkMax(DeviceIDs.pivotMotor, MotorType.kBrushless);
        intakeMotor = new SparkMax(DeviceIDs.intakeMotor, MotorType.kBrushless);
        // Init Pivot Components //
        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        pidController = pivotMotor.getClosedLoopController();
        // Configure Pivot Motor //
        pivotConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        pivotConfig.encoder
            .positionConversionFactor(1000)
            .velocityConversionFactor(1000);
        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(PID.p, PID.i, PID.d);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    // #region Action Methods //
    public void Intake()
    {
        intakeMotor.set(MotorSpeeds.intakeSpeed);
    }

    public void StopIntake()
    {
        intakeMotor.set(0);
    }

    public void Output()
    {
        intakeMotor.set(MotorSpeeds.outputSpeed);
    }
    // #endregion //
    // #region Base Methods //
    public void moveToPosition(double setPoint) {
        this.setPoint = setPoint;
    }

    public double getSetPoint() {
        return setPoint;
    }

    public double getPosition() {
        return pivotEncoder.getPosition();
    }

    public void run() {
        pidController.setReference(setPoint, ControlType.kPosition);
    }
    // #endregion
}