// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.DriveByJoysticks;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  public static final Joystick leftJoystick = new Joystick(0);
  public static final Joystick rightJoystick = new Joystick(0);

  public static DriveByJoysticks teleop = new DriveByJoysticks(() -> leftJoystick.getX(), () -> leftJoystick.getY(), () -> rightJoystick.getX(), () -> false);

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
