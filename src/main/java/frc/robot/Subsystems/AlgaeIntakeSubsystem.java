//written by Joachim and David//
package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.AlgaeIntake.*;
public class AlgaeIntakeSubsystem {
    //Motor Declaration//
    SparkMax IntakeMotor;
    SparkMax PivotMotor;

//Thingy//
     private double setPoint = Positions.Carry;

//PID Stuff//
    SparkAbsoluteEncoder Encoder;
    SparkClosedLoopController PidController;
    SparkMaxConfig Config;

//regular Values//
    public double MoveToShot;
    public double Walk;
    public double Grab;

   public AlgaeIntakeSubsystem(){
    //Regular Value Init//
    MoveToShot = 1;
    Walk = 2;
    Grab = 3;
    //init//
    IntakeMotor = new SparkMax(DeviceIDs.IntakeMotor, MotorType.kBrushless);
    PivotMotor = new SparkMax(DeviceIDs.PivotMotor, MotorType.kBrushless);

    //PID Init//
    Config = new SparkMaxConfig();

    Encoder = PivotMotor.getAbsoluteEncoder();

    PidController = PivotMotor.getClosedLoopController();

    //Setup Config//
    Config.inverted(true).idleMode(IdleMode.kBrake);
    Config.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(PID.P, PID.I, PID.D);
    PivotMotor.configure(Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

void ShiftPostition(double ShiftPos){
    setPoint = ShiftPos;
}

void KillSwitch(){ while(true){}}

   public void run() {
     PidController.setReference(setPoint, ControlType.kPosition);
   }
  
  public void intake(){
    IntakeMotor.set(Speeds.IntakeSpeed);
  }

  public void output(){
    IntakeMotor.set(Speeds.OutputSpeed);
  }

  public void stopIntake(){
    IntakeMotor.set(0);
  }
   
    public void ArticulateArmUp(){ //theoretically rotates the algae arm upwards
      PivotMotor.set(0.1);
      //TO DO: add if statements for limit switches so motor doesnt rotate unecessarily
    }

    public void ArticulateArmStop(){ //stops the movement of the algae pivot arm
        PivotMotor.set(0);
    }

    public void ArticulateArmDown(){ //theoretically rotates teh algae arm downwards
        PivotMotor.set(-0.1);
        //TO DO: add if statements for limit switches so motor doesnt rotate unecessarily
    }


      public void moveToShoot() 
      {
        ShiftPostition(MoveToShot);
      }
    
      public void moveToWalk() 
      {
        ShiftPostition(Walk);
      }
    
      public void moveToGrab() 
      {
        ShiftPostition(Grab);
      }
}