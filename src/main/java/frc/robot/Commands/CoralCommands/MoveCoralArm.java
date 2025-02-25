package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.Subsystems.*;
import java.time.Clock;

public class MoveCoralArm extends Command {
  Clock timer = Clock.systemDefaultZone();
  CoralIntakeSubsystem coralController;
  long timeoutTime;
  double setpoint;

  public MoveCoralArm(CoralIntakeSubsystem coralController, double setpoint) {
    this.coralController = coralController;
    this.setpoint = setpoint;

    addRequirements(coralController);
  }

  @Override
  public void initialize() {
    timeoutTime = timer.millis();
  }

  @Override
  public void execute() {
    coralController.moveToPosition(setpoint);
  }

  @Override
  public boolean isFinished() {
    return coralController.hasReachedSetpoint()
        || (timer.millis() - timeoutTime) > 5000; // cahnge 5 sec to constants
  }
}
