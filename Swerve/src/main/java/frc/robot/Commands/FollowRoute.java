package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class FollowRoute extends CommandBase{
    private double m_targetX;
    private PIDController m_xPidController;
    private double m_targetY;
    private PIDController m_yPidController;
    private double m_targetAngle;

    public FollowRoute(double targetX, double targetY, double targetAngle){
        m_targetX = targetX;
        m_targetY = targetY;
        m_targetAngle = targetAngle;
        m_xPidController = new PIDController(Consts.X_KP, Consts.X_KI, Consts.X_KD);
        m_yPidController = new PIDController(Consts.Y_KP, Consts.Y_KI, Consts.Y_KD);
    }

    @Override
    public void execute() {
        double xCurrent = Swerve.getInstance(Consts.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(Consts.USES_ABS_ENCODER).getY();

        double xOutput = MathUtil.clamp(m_xPidController.calculate(xCurrent, m_targetX), -1, 1);
        double yOutput = MathUtil.clamp(m_yPidController.calculate(yCurrent, m_targetY), -1, 1);
        SmartDashboard.putNumber("xOutput", xOutput);
        SmartDashboard.putNumber("yOutput", yOutput);
        Swerve.getInstance(Consts.USES_ABS_ENCODER).drive(new Vector2d(xOutput, yOutput), false);    
        Swerve.getInstance(Consts.USES_ABS_ENCODER).rotateTo(m_targetAngle);
    } 

    @Override
    public boolean isFinished() {
        double xCurrent = Swerve.getInstance(Consts.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(Consts.USES_ABS_ENCODER).getY();
        return (Math.abs(m_targetX) + Consts.X_THRESHOLD) > xCurrent &&
               (Math.abs(m_targetX) - Consts.X_THRESHOLD) < xCurrent &&
               (Math.abs(m_targetY) + Consts.Y_THRESHOLD) > yCurrent &&
               (Math.abs(m_targetY) - Consts.Y_THRESHOLD) < yCurrent;
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance(Consts.USES_ABS_ENCODER);
    }



    
}
