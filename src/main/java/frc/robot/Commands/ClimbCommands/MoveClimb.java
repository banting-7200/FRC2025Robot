package frc.robot.Commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.CageClimbSubsystem;

public class MoveClimb extends InstantCommand {
  // Subsystem //
  CageClimbSubsystem subsystem;
  // Instance Data //
  boolean isClimbing;

  // Constructor //
  public MoveClimb(CageClimbSubsystem subsystem, boolean isClimbing) {
    this.subsystem = subsystem;
    this.isClimbing = isClimbing;
    addRequirements(subsystem);
  }

  // Override Methods //
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (isClimbing) subsystem.climbUp();
    else subsystem.climbDown();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
