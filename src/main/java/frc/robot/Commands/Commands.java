// Folder //
package frc.robot.Commands;

// Imports //
import frc.robot.Subsystems.*;

// Command //
public class Commands {
  // Data //
  private AlgaeIntakeSubsystem algaeArm = AlgaeIntakeSubsystem.getInstance();
  private CoralIntakeSubsystem coralArm = CoralIntakeSubsystem.getInstance();
  private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
  private CageClimbSubsystem climber = CageClimbSubsystem.getInstance();

  // Constructor //
  public Commands() {}

  // #region Coral //
  public void intakeCoral() {
    if (!algaeArm.isUp) {
      if (!coralArm.hasCoral()) {
        elevator.moveToFloor();
        coralArm.moveToIntake();
        coralArm.intake();
      } else {
        elevator.moveToCarry();
        coralArm.stopIntake();
        coralArm.moveToCarry();
      }
    } else {
      algaeArm.armDown();
      intakeCoral();
      return;
    }
  }

  public void outputCoralAtPosition(double position) { // Non-Periodic //
    elevator.moveToPosition(position);
    // Checks //
    if (!algaeArm.isUp) { // If arm is down //
      coralArm.moveToDropoff();
    } else { // If arm is up //
      algaeArm.armDown();
      outputCoralAtPosition(position);
      return;
    }
  }

  // #endregion //
  // #region Algae //
  public void intakeAlgae() {
    // Conditions //
    if (algaeArm.hasAlgae()) return;
    // Move Arm Down to Intake //
    algaeArm.armDown();
    // Intake //
    algaeArm.Intake();
  }

  public void outputAlgaeAtPosition(double position) {
    // Move ELevator to Needed Position //
    elevator.moveToPosition(position);
    // Output //
    algaeArm.Output();
  }

  // #endregion //
  // #region Climber / Cage //
  public void toggleClimber() {
    climber.armToggle();
  }

  // #endregion //
  // #region Elevator //
  public void goLevelOne() {}

  // #endregion //
}
