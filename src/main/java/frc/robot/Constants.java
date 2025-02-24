package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

public final class Constants {
  public final class CageConstants {
    public final class Input {
      public static final int toggleCage = 0;
    }

    public final class DeviceIDz {
      public static final int climbMotor = 0;
    }

    public final class PID {
      public static final double p = 0;
      public static final double i = 0;
      public static final double d = 0;
    }

    public final class Positions {
      public static final double armUp = 0;
      public static final double armDown = 0;
    }
  }

  public final class Elevator {
    public static boolean mode = false; // Coral is false, Algae is true //

    public final class Input { // TODO: Improve Names //
      public static int floorLevel = 0;
      public static int levelOne = 0;
      public static int levelTwo = 0;
      public static int levelThree = 0;
      public static int levelFour = 0;
    }

    public final class MotorConfig {
      public static final boolean inverted = false;
      public static final int canID = 1;
      public static final double positionConversionFactor = 360;
      public static final double velocityConversionFactor = 1;
    }

    public final class PID {
      public static double p = 0.7;
      public static final double i = 0;
      public static final double d = 0;
    }

    public final class Positions {
      public static double coralOne = 0;
      public static double coralTwo = 0;
      public static double coralThree = 0;
      public static double coralFour = 0;
      public static double algaeOne = 0;
      public static double algaeTwo = 0;
      public static double floorLevel = 0;
      public static double carry = 0;
    }
  }

  public final class Algae {
    public final class MotorSpeeds {
      public static final double intakeSpeed = 1; // Should be positive //
      public static final double outputSpeed = 1; // Should be positive //
    }

    public final class DeviceIDs {
      public static final int intakeMotor = 0;
      public static final int pivotMotor = 1;
    }

    public final class PID {
      public static final double p = 0;
      public static final double i = 0;
      public static final double d = 0;
    }

    public final class Positions {
      public static final double armUp = 0;
      public static final double armDown = 0;
      public static final double safetyRange =
          0; // safteyRange is the range that the algae motor has to be within for the other arm to
      // be able to move up
    }

    public final class Input {
      public static final int intake = 0;
      public static final int output = 0;

      public static final int armUp = 0;
      public static final int armDown = 0;
      public static final int toggleArm = 0; // Toggles Arm to go up or down
      public static final int armArticulateUp = 0;
      public static final int armArticulateDown = 0;
    }
  }

  public final class Coral {
    public final class Input {
      public static final int intake = 0;
      public static final int output = 0;
      public static final int carryPosition = 0;
      public static final int intakePosition = 0;
      public static final int dropOffPosition = 0;

      public static final int armArticulateUp = 0;
      public static final int armArticulateDown = 0;
    }

    public final class MotorSpeeds {
      public static final double intakeSpeed = 1;
      public static final double outputSpeed = -1;
    }

    public final class DeviceIDs {
      public static final int intakeMotor = 0;
      public static final int pivotMotor = 1;
    }

    public final class Positions {
      public static final double carry = 0;
      public static final double intake = 0;
      public static final double dropOff = 0;
      public static final double safetyRange = 40; // TODO: please tune!!!!
      // safteyRange is the range that the arm motors have to be within for the other arm to be able
      // to move//
    }

    public final class PID {
      public static final double p = 0;
      public static final double i = 0;
      public static final double d = 0;
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
      public static final double slowP = 0.2;
      public static final double normalP = 0.7;
      public static double p = 0.7;

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

    public final class ButtonBox {
      public static final int port = 0;
    }
  }

  public final class Configurations {
    public static final int lightPort = 0;
    public static final int lightCount = 50;
  }
}
