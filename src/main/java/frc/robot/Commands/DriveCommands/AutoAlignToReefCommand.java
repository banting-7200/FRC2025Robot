package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.*;
import frc.robot.Vision.*;

public class AutoAlignToReefCommand extends Command {

  SwerveSubsystem swerve;
  Limelight limelight;

  public AutoAlignToReefCommand(SwerveSubsystem swerve, Limelight limelight) {
    this.swerve = swerve;
    this.limelight = limelight;
    addRequirements(swerve);
    addRequirements(limelight);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean in) {}
}
