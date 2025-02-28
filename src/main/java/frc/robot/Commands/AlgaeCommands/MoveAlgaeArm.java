package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlgaeIntakeSubsystem;
import java.time.Clock;

public class MoveAlgaeArm extends Command {
  AlgaeIntakeSubsystem algaeController;
  double setpoint;
  Clock timer = Clock.systemDefaultZone();
  long timeoutTime = 3000;

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
    if (algaeController.hasReachedSetpoint()) {
      System.out.println("Done");
      return true;
    }
    System.out.println(algaeController.getPosition());
    return timer.millis() > timeoutTime;
  }
}
