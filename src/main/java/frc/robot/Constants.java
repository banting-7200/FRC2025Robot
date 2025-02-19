package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

public final class Constants {

  public final class CageClimb{
    public final class DeviceIDz{
        public static final int climbMotor = 0;
    }
  }

  //AlgaeIntake likely incorrect
  public final class AlgaeIntake{
    public final class PID{
        public static final int P = 0;
        public static final int I = 0;
        public static final int D = 0;
    }
  }


  public final class DriveBase {
    public static final double maxSpeed = Units.feetToMeters(14.5); // metres per second
    // TODO: why this number?
    public static final double maxAngularVelocity = 5.627209491911525; // radians per second
    public static final double maxCreepSpeed = Units.feetToMeters(2); // metres per second
    public static final double maxCreepAngularVelocity = 2; // radians per second
    public static final double wheelDiameter = 4; // inches
    public static final double driveGearRatio = 6.75; // revolutions per wheel rotation
    // TODO: how to get is this number?
    public static final double angleGearRatio = 21.4285714286;
    // TODO: what is this?
    public static final double anglePulsesPerRotation = 360;

    // TODO: tune these
    public final class TranslationPID {
      public static final double p = 0.7;
      public static final double i = 0;
      public static final double d = 0;
    }

    // TODO: tune these
    public final class RotationPID {
      public static final double p = 0.4;
      public static final double i = 0;
      public static final double d = 0;
    }
  }

  public final class Field {
    private final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  }

  public final class Control {
    public final class Main {
      public static final int port = 0;
      public static final double leftXDeadband = 0.1;
      public static final double leftYDeadband = 0.1;
      public static final double rightXDeadband = 0.1;
      public static final double rightYDeadband = 0.1;
      public static final int zeroSwerveDriveButton = XboxController.Button.kA.value;
      public static final int enableCreepDrive = XboxController.Axis.kLeftTrigger.value;
    }
  }
}
