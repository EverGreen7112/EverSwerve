// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DriveByJoysticks;

import frc.robot.Utils.Consts;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  public static final Joystick leftJoystick = new Joystick(Consts.LEFT_JOYSTICK);
  public static final Joystick rightJoystick = new Joystick(Consts.RIGHT_JOYSTICK);
  public static final Joystick controller = new Joystick(2);
  public static final XboxController xbox = new XboxController(3);
  

  public static DriveByJoysticks teleop = new DriveByJoysticks(() -> controller.getX(), () -> controller.getY(), ()-> controller.getZ(), () -> true, true);
                                        //new DriveByJoysticks(() -> xbox.getLeftX(), () -> xbox.getLeftY(), () -> xbox.getRightX(), () -> true, true); 
  private void configureBindings() {
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
