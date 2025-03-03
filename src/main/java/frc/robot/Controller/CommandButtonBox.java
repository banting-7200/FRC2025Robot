package frc.robot.Controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandButtonBox extends CommandGenericHID {
  public Joystick controller;

  // Input Data //
  // assuming port count starts at zero, increase all ints by 1 if starts by 1
  public static final int port = 1; // unkown referance
  public static final int elevatorManualLift = 11;
  public static final int elevatorManualFall = 10;
  public static final int coralManualRotateLeft = 9;
  public static final int coralManualRotateRight = 8;
  public static final int intake = 7;
  public static final int output = 6;
  // Coral //
  public static final int level1Button = 3;
  public static final int level2Button = 2;
  public static final int level3Button = 1;
  // Algae //
  public static final int reZeroElevator = 5;
  public static final int floorLevelButton = 4;
  public static final int algaeLevel1 = 3;
  public static final int algaeLevel2 = 2;
  public static final int algaeNet = 1;

  public static final int gamePieceSwitch = 11;

  // Constructor //
  public CommandButtonBox(int port) {
    // Call Ancestor //
    super(port);
    // Initialize Data //
    this.controller = new Joystick(port);
  }

  // Input Methods //
  public Trigger elevatorManualLift() {
    return new JoystickButton(controller, elevatorManualLift);
  }

  public Trigger elevatorManualFall() {
    return new JoystickButton(controller, elevatorManualFall);
  }

  public Trigger coralManualRotateLeft() {
    return new JoystickButton(controller, coralManualRotateLeft);
  }

  public Trigger coralManualRotateRight() {
    return new JoystickButton(controller, coralManualRotateRight);
  }

  public Trigger intake() {
    return new JoystickButton(controller, intake);
  }

  public Trigger output() {
    return new JoystickButton(controller, output);
  }

  public Trigger level1Button() {
    return new JoystickButton(controller, level1Button);
  }

  public Trigger level2Button() {
    return new JoystickButton(controller, level2Button);
  }

  public Trigger level3Button() {
    return new JoystickButton(controller, level3Button);
  }

  public Trigger reZeroElevator() {
    return new JoystickButton(controller, reZeroElevator);
  }

  public Trigger floorLevelButton() {
    return new JoystickButton(controller, floorLevelButton);
  }

  public Trigger algaeLevel1() {
    return new JoystickButton(controller, algaeLevel1);
  }

  public Trigger algaeLevel2() {
    return new JoystickButton(controller, algaeLevel2);
  }

  public Trigger algaeNet() {
    return new JoystickButton(controller, algaeNet);
  }

  public Trigger gamePieceSwitch() {
    return new JoystickButton(controller, gamePieceSwitch);
  }
}
