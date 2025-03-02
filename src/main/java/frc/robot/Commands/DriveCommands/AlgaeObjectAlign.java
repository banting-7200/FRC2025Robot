package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Vision.PhotonVisionCamera;
import java.util.function.Supplier;

public class AlgaeObjectAlign extends Command {

  private SwerveSubsystem swerveSubsystem = null;
  private final PhotonVisionCamera photonCam;

  private final PIDController positionController;
  private final PIDController rotationController;

  private double d_algaeArea;
  private double c_algaeArea;

  private Command s_command;

  private Pose2d initialPose2d;

  public final Supplier<double[]> leftJoystick;

  public final Supplier<double[]> rightJoystick;

  //   public AlgaeObjectAlign() // To stop errors...
  //       {
  //     swerveSubsystem = null;
  //     leftJoystick = null;
  //     photonCam = null;
  //     positionController = null;
  //     rightJoystick = null;
  //     rotationController = null;
  //   }

  public AlgaeObjectAlign(
      SwerveSubsystem swerveSubsystem,
      PhotonVisionCamera photonCam,
      Supplier<double[]> leftJoystick,
      Supplier<double[]> rightJoystick) {
    //
    this.swerveSubsystem = swerveSubsystem;
    this.photonCam = photonCam;

    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;

    positionController = new PIDController(1, 0, 0);
    positionController.setSetpoint(d_algaeArea);

    rotationController = new PIDController(0.035, 0.0001, 0);

    rotationController.setSetpoint(0);
    rotationController.setTolerance(2, 4);

    addRequirements(swerveSubsystem, photonCam);
  }

  @Override
  public void initialize() {
    /*
     * initialPose2d = swerveSubsystem.getPose();
     */ }

  @Override
  public void execute() {

    System.out.println("Current yaw: " + photonCam.getTargetYaw());
    double rotationAdjust = 0;
    if (photonCam.hasTarget()) {
      c_algaeArea = photonCam.getTargetArea();
      rotationAdjust = rotationController.calculate(photonCam.getTargetYaw(), 0);
      swerveSubsystem.drive(
          new Translation2d(
              MathUtil.applyDeadband(-leftJoystick.get()[1], 0.1) * 1.5,
              MathUtil.applyDeadband(-leftJoystick.get()[0], 0.1) * 1.5),
          rotationAdjust,
          false);
    } else {
      /*
       * swerveSubsystem.driveFieldOriented(leftJoystick.get(), rightJoystick.get());
       */
      swerveSubsystem.drive(leftJoystick.get(), rightJoystick.get());
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    /*
     * swerveSubsystem.setOdometry(initialPose2d);
     */
    swerveSubsystem.lock();
    if (!interrupted) {
      System.out.println("Ended Note Align successfully");
    } else {
      System.out.println("Interrupted Note Align Commmand");
    }
  }
}
