package frc.robot.Commands.CoralCommands;

import java.time.Clock;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.coralSystem;
import frc.robot.Subsystems.CoralIntakeSubsystem;

public class OutputCoralCommand extends Command {
    CoralIntakeSubsystem coralController;
    Clock timer = Clock.systemDefaultZone();
    long timeoutTime;
    public OutputCoralCommand(CoralIntakeSubsystem coralController){
        this.coralController = coralController;
        addRequirements(coralController);
    }

    @Override
  public void initialize() {
    timeoutTime = timer.millis();
  }

  @Override
  public void execute() {
    coralController.spinIntake(coralSystem.MotorSpeeds.outputSpeed);
    
  }

  @Override
  public boolean isFinished() {
    return (timer.millis() - timeoutTime) >= 2000;
  }

  @Override
  public void end(boolean interupted){
    coralController.stopIntake();
  }


}
