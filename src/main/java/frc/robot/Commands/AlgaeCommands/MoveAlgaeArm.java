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
    System.out.println("Start");
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println("moving to algae position");
    algaeController.moveToPosition(setpoint);
    algaeController.run();
  }

  @Override
  public boolean isFinished() {
    System.out.println("Done");
    return algaeController.hasReachedSetpoint();
  }
}
