package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandTimes;
import frc.robot.Subsystems.CoralIntakeSubsystem;
import java.time.Clock;

public class IntakeCoralCommand extends Command {
  CoralIntakeSubsystem coralController;
  Clock timer = Clock.systemDefaultZone();
  long timeoutTime;

  public IntakeCoralCommand(CoralIntakeSubsystem coralController) {
    this.coralController = coralController;
    addRequirements(coralController);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    coralController.intake();
  }

  @Override
  public boolean isFinished() {
    return coralController.hasCoral()
        || (timer.millis() - timeoutTime) >= CommandTimes.coralIntakeTime;
  }

  @Override
  public void end(boolean interupted) {
    coralController.stop();
  }
}
