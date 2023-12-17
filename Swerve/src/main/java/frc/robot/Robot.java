// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private Swerve m_swerveInstance;


  @Override
  public void robotInit() {
    m_swerveInstance = Swerve.getInstance(Consts.USES_ABS_ENCODER);
    m_robotContainer = new RobotContainer();    
    SmartDashboard.putNumber("max speed", 1);
    SmartDashboard.putNumber("max angular speed", 5);
    SmartDashboard.putNumber("heading kp", 0.034);
    SmartDashboard.putNumber("heading kd", 0.0);
    m_swerveInstance.zeroModulesAngles();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("top right angle", m_swerveInstance.getModule(0).getAngle());
    SmartDashboard.putNumber("top left angle", m_swerveInstance.getModule(1).getAngle());
    SmartDashboard.putNumber("down right angle", m_swerveInstance.getModule(2).getAngle());
    SmartDashboard.putNumber("down left angle", m_swerveInstance.getModule(3).getAngle());

    SmartDashboard.putNumber("top right velocity", m_swerveInstance.getModule(0).getSpeed());
    SmartDashboard.putNumber("top left velocity", m_swerveInstance.getModule(1).getSpeed());
    SmartDashboard.putNumber("down right velocity", m_swerveInstance.getModule(2).getSpeed());
    SmartDashboard.putNumber("down left velocity", m_swerveInstance.getModule(3).getSpeed());

  }

  @Override
  public void disabledInit() {
    Swerve.getInstance(true).stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.teleop.schedule();
    // for(int i =0 ; i < 4;i++){
    //    Swerve.getInstance(true).m_modules[i].setState(0.1, 90);
    // }
    m_swerveInstance.zeroYaw();
  }

  @Override
  public void teleopPeriodic() {}

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
  public void testPeriodic() {}

  @Override
  public void testExit() {
    Swerve.getInstance(true).stop();
  }
}
