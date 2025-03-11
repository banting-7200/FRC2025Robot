package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class TankDrive extends Command {
  // Instance Data //
  SwerveSubsystem swerve;
  Supplier<double[]> leftJoystick;
  Supplier<double[]> rightJoystick;

  // Constructor //
  public TankDrive(
      SwerveSubsystem swerve, Supplier<double[]> leftJoystick, Supplier<double[]> rightJoystick) {
    this.swerve = swerve;
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
  }

  // Override Methods //
  @Override
  public void execute() {
    // Drive //
    swerve.drive(
        new Translation2d(
            MathUtil.applyDeadband(-leftJoystick.get()[1], 0.1) * 2.5,
            MathUtil.applyDeadband(leftJoystick.get()[0], 0.1) * 2.5),
        0,
        false);
  }
}
