// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Constants;

public class Robot extends TimedRobot implements Constants {
  private Swerve m_swerveInstance;

  @Override
  public void robotInit() {
    m_swerveInstance = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER);
    new RobotContainer();
    SmartDashboard.putNumber("max drive speed", 1);
    SmartDashboard.putNumber("max angular speed", 1.5);
    SmartDashboard.putNumber("heading kp", 0.03);
    SmartDashboard.putNumber("heading kd", 0.0001);
    SmartDashboard.putNumber("xkp", 1.5);
    SmartDashboard.putNumber("xkd", 0);
    SmartDashboard.putNumber("ykp", 1.5);
    SmartDashboard.putNumber("ykd", 0);
    m_swerveInstance.zeroModulesAngles();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    Swerve.getInstance(true).stop();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    m_swerveInstance.zeroYaw();
    Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).resetOdometry();
    Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).resetOdometry();
    RobotContainer.teleop.schedule();
    // for(int i =0 ; i < 4;i++){
    // Swerve.getInstance(true).m_modules[i].setState(0.1, 90);
    // }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
    Swerve.getInstance(true).stop();
  }

  @Override
  public void testInit() {
    Swerve.getInstance(true).zeroYaw();
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.teleop.schedule();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
    Swerve.getInstance(true).stop();
  }
}
