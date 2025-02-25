package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.coralSystem;
import frc.robot.Subsystems.CoralIntakeSubsystem;

public class IntakeCoralCommand extends Command {
  CoralIntakeSubsystem coralController;

  public IntakeCoralCommand(CoralIntakeSubsystem coralController) {
    this.coralController = coralController;
    addRequirements(coralController);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    coralController.spinIntake(coralSystem.MotorSpeeds.intakeSpeed);
  }

  @Override
  public boolean isFinished() {
    return coralController.hasCoral();
  }

  @Override
  public void end(boolean interupted){
    coralController.stopIntake();
  }
}
