// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DriveByJoysticks;
import frc.robot.Commands.FollowRoute;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Constants;
import frc.robot.Utils.SwervePoint;

public class RobotContainer implements Constants {

  public RobotContainer() {
    configureBindings();
  }

  public static final Joystick leftJoystick = new Joystick(JoystickValues.LEFT_JOYSTICK);
  public static final Joystick rightJoystick = new Joystick(JoystickValues.RIGHT_JOYSTICK);
  public static final Joystick controller = new Joystick(JoystickValues.OPERATOR);
  public static final XboxController xbox = new XboxController(3);

  public static DriveByJoysticks teleop = new DriveByJoysticks(() -> controller.getX(), () -> controller.getY(),
      () -> controller.getZ(), () -> true, SwerveValues.USES_ABS_ENCODER);
  // new DriveByJoysticks(() -> xbox.getLeftX(), () -> xbox.getLeftY(), () ->
  // xbox.getRightX(), () -> true, true);
  // new DriveByJoysticks(() -> leftJoystick.getX(), () -> leftJoystick.getY(),
  // ()-> rightJoystick.getX(), () -> true, Consts.USES_ABS_ENCODER);

  private void configureBindings() {

    Trigger rotateRobotBy45 = new JoystickButton(controller, 6).onTrue(new InstantCommand(() -> {
      Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).rotateBy(45);
    }));

    Trigger rotateRobotByMinus45 = new JoystickButton(controller, 5).onTrue(new InstantCommand(() -> {
      Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).rotateBy(-45);
      ;
    }));
    Trigger rotateRobot180 = new JoystickButton(controller, 8).onTrue(new InstantCommand(() -> {
      Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).rotateBy(180);
      ;
    }));
    Trigger rotateRobotTo0 = new JoystickButton(controller, 7).onTrue(new InstantCommand(() -> {
      Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).rotateTo(0);
      ;
    }));
    Trigger resetOdometry = new JoystickButton(controller, 3).onTrue(new InstantCommand(() -> {
      Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).resetOdometry();
    }));

    ArrayList<Point> posList = new ArrayList<Point>();
    posList.add(new Point(1, 1));
    Trigger followRoute = new JoystickButton(controller, 10).onTrue(new FollowRoute(posList, 90));

    // Trigger savePoint = new JoystickButton(controller, 2).onTrue(new InstantCommand(() -> {
    //   posList.add(new SwervePoint(Swerve.getInstance(Constants.SwerveValues.USES_ABS_ENCODER).getX(),
    //                               Swerve.getInstance(Constants.SwerveValues.USES_ABS_ENCODER).getY(),
    //                               Swerve.getInstance(Constants.SwerveValues.USES_ABS_ENCODER).getGyro().getAngle()));
    // }));

    // Trigger removePoint = new JoystickButton(controller, 1).onTrue(new InstantCommand(() -> {
    //   posList.clear();
    // }));

  
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
