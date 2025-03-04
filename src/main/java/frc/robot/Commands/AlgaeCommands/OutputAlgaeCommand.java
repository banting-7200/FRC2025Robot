package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevator;
import frc.robot.Subsystems.AlgaeIntakeSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import java.time.Clock;

public class OutputAlgaeCommand extends Command {
  AlgaeIntakeSubsystem algaeController;
  ElevatorSubsystem elevatorController;
  Clock timer = Clock.systemDefaultZone();
  boolean isProcessor;
  long timeoutTime;

  public OutputAlgaeCommand(
      AlgaeIntakeSubsystem algaeController, ElevatorSubsystem elevatorController) {
    this.algaeController = algaeController;
    this.elevatorController = elevatorController;
    addRequirements(algaeController);
  }

  @Override
  public void initialize() {
    timeoutTime = timer.millis();
    this.isProcessor = elevatorController.getPosition() == Elevator.Positions.processorLevel;
  }

  @Override
  public void execute() {
    if (isProcessor) algaeController.outputProcessor();
    else algaeController.output();
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
