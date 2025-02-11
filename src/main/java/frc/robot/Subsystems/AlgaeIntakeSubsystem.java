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

import frc.robot.RobotContainer;
import frc.robot.Constants.Algae.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
// Subsystem //
public class AlgaeIntakeSubsystem
{
    // Instance Data //
    private double setPoint;
    private boolean armState;
    // Motors //
    private SparkMax pivotMotor;
    private SparkMax intakeMotor;
    // Motor Config //
    private SparkMaxConfig pivotConfig = new SparkMaxConfig();
    // Pivot Components //
    private SparkAbsoluteEncoder pivotEncoder;
    private SparkClosedLoopController pidController;
    // Input Controllers //
    private Joystick controller = new Joystick(0);
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
    // #region Base Methods //
    /**
     * Sets the target angle of the motor
     * @param setPoint The angle
     */
    public void moveToPosition(double setPoint) {
        this.setPoint = setPoint;
    }

    /**
     * Gets the target angle of the motor
     * @return the current angle of the motor
     */
    public double getSetPoint() {
        return setPoint;
    }

    /**
     * Gets the precise angle of the motor right now
     * @return The angle of the motor
     */
    public double getPosition() {
        return pivotEncoder.getPosition();
    }

    /**
     * For handling input, not used yet though...
     */
    public void handleInput() {
        // Data //
        EventLoop loop = RobotContainer.loop;
        // Events //
        BooleanEvent armToggleEvent =
        new BooleanEvent(loop, controller.button(0, loop)) // Arms //
            // debounce for more stability
            .debounce(0.2);

        // if we're at the target velocity, kick the ball into the shooter wheel
        armToggleEvent.ifHigh(() -> 
        {
            ArmToggle();
        });
    }

    /**
     * Update method
     */
    public void run() {
        pidController.setReference(setPoint, ControlType.kPosition);
    }
    // #endregion
    // #region Action Methods //
    /**
     * Intake Algae
     */
    public void Intake()
    {
        intakeMotor.set(MotorSpeeds.intakeSpeed);
    }

    /**
     * Stop the intake motor
     */
    public void StopIntake()
    {
        intakeMotor.set(0);
    }

    /**
     * Spit out algae
     */
    public void Output()
    {
        intakeMotor.set(MotorSpeeds.outputSpeed);
    }
    // Arm Controls //
    /**
     * Moves the arm up
     */
    public void ArmUp()
    {
        moveToPosition(Positions.armUp);
        armState = true;
    }
    
    /**
     * Moves the arm down
     */
    public void ArmDown()
    {
        moveToPosition(Positions.armDown);
        armState = false;
    }

    /**
     * Toggles between arm up and arm down
     */
    public void ArmToggle()
    {
        // Checks //
        if (armState)
            ArmDown();
        else
            ArmUp();
        // Settings //
        armState = !armState;
    }
    // #endregion //
}