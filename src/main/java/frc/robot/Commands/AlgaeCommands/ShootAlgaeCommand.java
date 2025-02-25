package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.CommandTimes;
import frc.robot.Subsystems.AlgaeIntakeSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import java.time.Clock;

public class ShootAlgaeCommand extends Command {
  AlgaeIntakeSubsystem algaeController;
  ElevatorSubsystem elevator;
  Clock timer = Clock.systemDefaultZone();
  long timeoutTime;

  public ShootAlgaeCommand(AlgaeIntakeSubsystem algaeController, ElevatorSubsystem elevator) {
    this.algaeController = algaeController;
    this.elevator = elevator;
    addRequirements(algaeController);
  }

  @Override
  public void initialize() {
    timeoutTime = timer.millis();
    algaeController.moveToShootPosition();
    elevator.moveToPosition(Constants.Elevator.Positions.net);
  }

  @Override
  public void execute() {
    if (algaeController.hasReachedSetpoint() && elevator.hasReachedSetpoint())
      algaeController.output();
  }

  @Override
  public boolean isFinished() {
    return (timer.millis() - timeoutTime) >= CommandTimes.algaeShootTime;
  }

  @Override
  public void end(boolean interupted) {
    if (!interupted) algaeController.moveToUpPosition();
    algaeController.stop();
  }
}
