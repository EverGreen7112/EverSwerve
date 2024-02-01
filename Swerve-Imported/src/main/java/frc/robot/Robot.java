// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Vision;

public class Robot extends TimedRobot implements Constants {
  private Swerve m_swerveInstance;
  private Field2d m_field;
  private Vision m_vision;
  
  @Override
  public void robotInit() {
    m_swerveInstance = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER);
    new RobotContainer();
    SmartDashboard.putNumber("max drive speed", 1);
    SmartDashboard.putNumber("max angular speed", 200);
    m_swerveInstance.zeroModulesAngles();
    //create and add robot field data to dashboard
    m_field = new Field2d();
    SmartDashboard.putData(m_field);
    m_vision = new Vision(VisionValues.LOCALIZATION_VISION_PORT);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //get current position and rotation of robot 
    double xCurrent = m_swerveInstance.getX();
    double yCurrent = m_swerveInstance.getY();
    double headingCurrent = m_swerveInstance.getAngleWithOffset();
    //update the robot position of dashboard
    m_field.setRobotPose(yCurrent, xCurrent, new Rotation2d(Math.toRadians(-headingCurrent)));
  }

  @Override
  public void disabledInit() {
    m_swerveInstance.stop();
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
    m_swerveInstance.resetOdometry();
    m_swerveInstance.setModulesToAbs();
    RobotContainer.teleop.schedule();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
    m_swerveInstance.stop();
  }

  @Override
  public void testInit() {
    m_swerveInstance.zeroYaw();
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.teleop.schedule();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
    m_swerveInstance.stop();
  }
}
