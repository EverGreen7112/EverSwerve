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
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    Swerve.getInstance(true).initModulesToAbs();
    SmartDashboard.putNumber("max speed", 0.2);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("top right angle", Consts.modulo(Swerve.getInstance(true).m_modules[0].getPos(), 360 ));
    SmartDashboard.putNumber("top left angle", Consts.modulo(Swerve.getInstance(true).m_modules[1].getPos(), 360 ));
    SmartDashboard.putNumber("down right angle", Consts.modulo(Swerve.getInstance(true).m_modules[2].getPos(), 360 ));
    SmartDashboard.putNumber("down left angle", Consts.modulo(Swerve.getInstance(true).m_modules[3].getPos(), 360 ));
    SmartDashboard.putNumber("angle", Swerve.getInstance(true).getGyro().getAngle());
    SmartDashboard.putNumber("controller angle", Consts.modulo(RobotContainer.controller.getDirectionDegrees(), 360));
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
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    // Swerve.getInstance(true).zeroYaw();    
    RobotContainer.teleop.schedule();
    // for(int i =0 ; i < 4;i++){
    //    Swerve.getInstance(true).m_modules[i].setState(0.1, 90);
    // }
  }

  @Override
  public void teleopPeriodic() {
    // Swerve.getInstance(true).m_modules[1].setState(0.01, RobotContainer.controller.getDirectionDegrees());
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
  public void testPeriodic() {}

  @Override
  public void testExit() {
    Swerve.getInstance(true).stop();
  }
}
