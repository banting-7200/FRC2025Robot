package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class MoveElevator extends Command {
  ElevatorSubsystem elevator;
  double setpoint;

  public MoveElevator(ElevatorSubsystem elevator, double setpoint) {
    this.elevator = elevator;
    this.setpoint = setpoint;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevator.moveToPosition(setpoint);
  }

  @Override
  public boolean isFinished() {
    return elevator.hasReachedSetpoint();
  }
}
