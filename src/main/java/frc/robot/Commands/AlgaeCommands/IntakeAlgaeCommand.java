package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlgaeIntakeSubsystem;
import java.time.Clock;

public class IntakeAlgaeCommand extends Command {
  AlgaeIntakeSubsystem algaeController;
  Clock timer = Clock.systemDefaultZone();
  long timeoutTime;

  public IntakeAlgaeCommand(AlgaeIntakeSubsystem algaeController) {
    this.algaeController = algaeController;
    addRequirements(algaeController);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println("intaking algae");
    algaeController.moveToDownPosition();
    algaeController.intake();
  }

  @Override
  public boolean isFinished() {
    return algaeController.hasAlgae();
    // || (timer.millis() - timeoutTime) >= CommandTimes.algaeIntakeTime;
  }

  @Override
  public void end(boolean intrupted) {
    algaeController.stop();
  }
}
