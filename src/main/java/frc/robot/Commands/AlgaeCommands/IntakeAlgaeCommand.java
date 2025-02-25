package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.algaeSystem;
import frc.robot.Subsystems.AlgaeIntakeSubsystem;

public class IntakeAlgaeCommand extends Command {
  AlgaeIntakeSubsystem algaeController;

  public IntakeAlgaeCommand(AlgaeIntakeSubsystem algaeController) {
    this.algaeController = algaeController;
    addRequirements(algaeController);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    algaeController.spinIntake(algaeSystem.MotorSpeeds.intakeSpeed);;
  }

  @Override
  public boolean isFinished() {
    return algaeController.hasAlgae();
  }

  @Override
  public void end(boolean intrupted){
    algaeController.StopIntake();
  }
}
