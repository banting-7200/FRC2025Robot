package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlgaeIntakeSubsystem;

public class MoveAlgaeArm extends Command {
  AlgaeIntakeSubsystem algaeController;
  double setpoint;

  public MoveAlgaeArm(AlgaeIntakeSubsystem algaeController, double setpoint) {
    this.algaeController = algaeController;
    this.setpoint = setpoint;
    addRequirements(algaeController);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    algaeController.moveToPosition(setpoint);
  }

  @Override
  public boolean isFinished() {
    return algaeController.hasReachedSetpoint();
  }
}
