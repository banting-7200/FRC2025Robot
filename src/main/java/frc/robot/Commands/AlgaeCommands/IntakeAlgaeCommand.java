package frc.robot.Commands.AlgaeCommands;

import java.time.Clock;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeSystem;
import frc.robot.Constants.CommandTimes;
import frc.robot.Subsystems.AlgaeIntakeSubsystem;

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
    algaeController.moveToDownPosition();
    algaeController.intake();
  }

  @Override
  public boolean isFinished() {
    return algaeController.hasAlgae() || (timer.millis() - timeoutTime) >= CommandTimes.algaeIntakeTime;
  }

  @Override
  public void end(boolean intrupted) {
    algaeController.stop();
  }
}
