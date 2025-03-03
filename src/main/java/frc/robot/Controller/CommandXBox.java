package frc.robot.Controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandXBox extends CommandGenericHID {
  // Physical Controller //
  public CommandXboxController controller;
  // Input Data //
  public final int algaeAlign = Axis.kRightTrigger.value;
  public final int cageClimbUp = 0;
  public final int cageClimbDown = 180;

  public static final int zeroSwerveDriveButton = XboxController.Button.kA.value;
  public static final int enableCreepDrive = XboxController.Axis.kLeftTrigger.value;
  public static final int switchTestMode = XboxController.Button.kY.value;

  // Constructor //
  public CommandXBox(int port) {
    // Call Ancestor Constructor //
    super(port);
    // Initialize Controller //
    this.controller = new CommandXboxController(port);
  }

  // Input Methods //
  public Trigger cageClimbUp() {
    return controller.pov(cageClimbUp);
  }

  public Trigger cageClimbDown() {
    return controller.pov(cageClimbDown);
  }

  public Trigger algaeAlign() {
    return controller.axisGreaterThan(algaeAlign, .5);
  }

  // Drive //

  public Trigger enableCreepDrive() {
    return controller.axisGreaterThan(enableCreepDrive, .5);
  }

  public Trigger zeroSwerveDrive() {
    return controller.button(zeroSwerveDriveButton);
  }
}
