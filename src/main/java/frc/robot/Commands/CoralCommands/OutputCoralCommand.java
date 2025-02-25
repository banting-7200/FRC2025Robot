package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandTimes;
import frc.robot.Subsystems.CoralIntakeSubsystem;
import java.time.Clock;

public class OutputCoralCommand extends Command {
  CoralIntakeSubsystem coralController;
  Clock timer = Clock.systemDefaultZone();
  long timeoutTime;

  public OutputCoralCommand(CoralIntakeSubsystem coralController) {
    this.coralController = coralController;
    addRequirements(coralController);
  }

  @Override
  public void initialize() {
    timeoutTime = timer.millis();
  }

  @Override
  public void execute() {
    coralController.intake();
  }

  @Override
  public boolean isFinished() {
    return (timer.millis() - timeoutTime) >= CommandTimes.coralShootTime;
  }

  @Override
  public void end(boolean interupted) {
    coralController.stop();
  }
}
