// Folder //
package frc.robot.Commands.ElevatorCommands;

// Imports //
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevator;
import frc.robot.Subsystems.ElevatorSubsystem;

// Command //
public class ZeroElevator extends Command {
  ElevatorSubsystem elevator;

  public ZeroElevator(ElevatorSubsystem elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Data //

  @Override
  public void execute() {
    elevator.setMotorSpeed(Elevator.reZeroSpeed);
    elevator.run();
  }

  @Override
  public boolean isFinished() {
    return elevator.bottomLimitSwitchPressed();
  }

  @Override
  public void end(boolean in) {
    elevator.stopMotor();
  }
}
