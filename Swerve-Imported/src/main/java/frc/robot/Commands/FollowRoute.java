package frc.robot.Commands;

import java.util.ArrayList;

import org.opencv.core.Point;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Funcs;
import frc.robot.Utils.SwervePoint;
import frc.robot.Utils.Vector2d;

public class FollowRoute extends CommandBase implements Constants {
     private PIDController m_xPidController;
    private PIDController m_yPidController;
    private double m_targetAngle;
    private ArrayList<SwervePoint> m_posList;

    public FollowRoute(ArrayList<SwervePoint> posList) {
        addRequirements(Swerve.getInstance(SwerveValues.USES_ABS_ENCODER));
        if (posList.isEmpty())
            return;
        m_posList = posList;
        m_xPidController = new PIDController(PIDValues.X_KP, PIDValues.X_KI, PIDValues.X_KD);
        m_yPidController = new PIDController(PIDValues.Y_KP, PIDValues.Y_KI, PIDValues.Y_KD);
    }

    @Override
    public void execute() {
        if (m_posList.isEmpty())
            return;
        //get current values
        double xCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getY();
        SmartDashboard.putNumber("target x", m_posList.get(0).getX());
        SmartDashboard.putNumber("target y", m_posList.get(0).getY());
        SmartDashboard.putNumber("target theta", m_posList.get(0).getAngle());

        //get outputs
        double xOutput = MathUtil.clamp(m_xPidController.calculate(xCurrent, m_posList.get(0).getX()), -1, 1);
        double yOutput = MathUtil.clamp(m_yPidController.calculate(yCurrent, m_posList.get(0).getY()), -1, 1);
        SmartDashboard.putNumber("xOutput", xOutput);
        SmartDashboard.putNumber("yOutput", yOutput);
        Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).drive(new Vector2d(xOutput, yOutput), true);
        Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).rotateTo(m_targetAngle);
    }

    @Override
    public boolean isFinished() {
        if (m_posList.isEmpty())
            return true;
        double xCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getY();
        if ((Math.abs(m_posList.get(0).getX()) + PIDValues.X_THRESHOLD) > xCurrent &&
                (Math.abs(m_posList.get(0).getX()) - PIDValues.X_THRESHOLD) < xCurrent &&
                (Math.abs(m_posList.get(0).getY()) + PIDValues.Y_THRESHOLD) > yCurrent &&
                (Math.abs(m_posList.get(0).getY()) - PIDValues.Y_THRESHOLD) < yCurrent) {
            m_posList.remove(0);
            return m_posList.isEmpty();
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).stop();
    }

}
