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
  public void initialize() {
    System.out.println("Moving elevator to " + setpoint);
    elevator.moveToPosition(setpoint);
  }

  @Override
  public void execute() {
    System.out.println("Current Elevator Position: " + setpoint);
    elevator.run();
  }

  @Override
  public boolean isFinished() {
    if (elevator.hasReachedSetpoint()) {
      System.out.println("Done Elevator");
      return true;
    }
    return false;
  }
}
