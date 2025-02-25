package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

public final class Constants {
  public final class climber {

    public final class PID {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
    }

    public final class Positions {
      public static final double armUp = 0;
      public static final double armDown = 0;
    }
  }

  public final class Elevator {

    public final class MotorConfig {
      public static final boolean inverted = true;
      public static final int canID = 1;
      public static final double positionConversionFactor = 360;
      public static final double velocityConversionFactor = 1;
    }

    public final class PID {
      public static double P = 0.7;
      public static final double I = 0;
      public static final double D = 0;
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

  public final class algaeSystem {
    public final class MotorSpeeds {
      public static final double intakeSpeed = 1; // Should be positive //
      public static final double outputSpeed = 1; // Should be positive //
    }

    public final class PID {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
    }

    public final class Positions {
      public static final double armUp = 0;
      public static final double armDown = 0;
      public static final double safetyRange =
          0; // safteyRange is the range that the algae motor has to be within for the other arm to
      // be able to move up
    }
  }

  public final class coralSystem {
    public final class MotorSpeeds {
      public static final double intakeSpeed = 1;
      public static final double outputSpeed = -1;
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
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
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
      public static final int port = 1;
      public static final int elevatorManualLift = 10;
      public static final int elevatorManualFall = 11;
      public static final int coralManualRotateLeft = 11;
      public static final int coralManualRotateRight = 11;
      public static final int intake = 11;
      public static final int output = 11;
      public static final int floorLevelButton = 11;
      public static final int level1Button = 11;
      public static final int level2Button = 11;
      public static final int level3Button = 11;
      public static final int level4Button = 11;
      public static final int gamePieceSwitch = 11;
    }
  }

  public final class Configurations {
    public static final int lightPort = 0;
    public static final int lightCount = 50;
  }

  public final class deviceIDs {
    public static final int elevatorID = 0;
    public static final int coralPivotID = 0;
    public static final int coralIntakeID = 0;
    public static final int algaePivotID = 0;
    public static final int algaeIntakeID = 0;
    public static final int climberID = 0;
  }
}
