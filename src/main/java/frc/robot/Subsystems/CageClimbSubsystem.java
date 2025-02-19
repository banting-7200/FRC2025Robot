package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.AlgaeIntake.PID;
import frc.robot.Constants.CageClimb.*;

public class CageClimbSubsystem{
    SparkMax climbMotor;
    SparkMaxConfig Config;
    SparkAbsoluteEncoder Encoder;
    SparkClosedLoopController PidController;

    double extend;
    double retract;

    public CageClimbSubsystem(){
      extend = 1;
      retract = 1;
      
      climbMotor = new SparkMax(DeviceIDz.climbMotor, MotorType.kBrushless);
      
      Config = new SparkMaxConfig();
      Encoder = climbMotor.getAbsoluteEncoder();
      PidController = climbMotor.getClosedLoopController();

    Config.inverted(true).idleMode(IdleMode.kBrake);
    Config.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(PID.P, PID.I, PID.D);
    climbMotor.configure(Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }



    void KillSwitch(){ while(true){}}

   
    public void ArticulateArmUp(){ //theoretically rotates the climb arm upwards
      climbMotor.set(0.1);
      //TO DO: add if statements for limit switches so motor doesnt rotate unecessarily
    }

    public void ArticulateArmStop(){ //stops the movement of the climb pivot arm
        climbMotor.set(0);
    }

    public void ArticulateArmDown(){ //theoretically rotates the climb arm downwards
        climbMotor.set(-0.1);
        //TO DO: add if statements for limit switches so motor doesnt rotate unecessarily
    }
}