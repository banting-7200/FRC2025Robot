package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlgaeIntakeSubsystem;
import java.time.Clock;

public class OutputAlgaeCommand extends Command {
  AlgaeIntakeSubsystem algaeController;
  Clock timer = Clock.systemDefaultZone();
  long timeoutTime;

  public OutputAlgaeCommand(AlgaeIntakeSubsystem algaeController) {
    this.algaeController = algaeController;
    addRequirements(algaeController);
  }

  @Override
  public void initialize() {
    timeoutTime = timer.millis();
  }

  @Override
  public void execute() {
    algaeController.output();
  }

  @Override
  public boolean isFinished() {
    return (timer.millis() - timeoutTime) >= 2000;
  }

  @Override
  public void end(boolean interupted) {
    algaeController.stop();
  }
}
