package frc.robot.Commands;

import java.util.ArrayList;

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
    private ArrayList<SwervePoint> m_posList;
    private int current;

    public FollowRoute(ArrayList<SwervePoint> posList) {
        m_posList = posList;
        m_xPidController = new PIDController(PIDValues.X_KP, PIDValues.X_KI, PIDValues.X_KD);
        m_yPidController = new PIDController(PIDValues.Y_KP, PIDValues.Y_KI, PIDValues.Y_KD);
    }

    @Override
    public void initialize() {
        // inorder to set MAX_ANGULAR_SPEED to auto value
        SmartDashboard.putNumber("max angular speed", SpeedValues.AUTONOMOUS_MAX_ANGULAR_SPEED);
        current = 0;
    }

    @Override
    public void execute() {
        if (m_posList.isEmpty())
            return;
        //get current position
        double xCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getY();
        //calculate outputs
        double xOutput = MathUtil.clamp(m_xPidController.calculate(xCurrent, m_posList.get(current).getX()), -1, 1);
        double yOutput = MathUtil.clamp(m_yPidController.calculate(yCurrent, m_posList.get(current).getY()), -1, 1);
        //round values
        xCurrent = Funcs.roundAfterDecimalPoint(xCurrent, 2);
        yCurrent = Funcs.roundAfterDecimalPoint(yCurrent, 2);
        //apply outputs
        Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).drive(new Vector2d(xOutput, yOutput), false);
        Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).rotateTo(m_posList.get(current).getAngle());
    }

    @Override
    public boolean isFinished() {
        if (m_posList.isEmpty())
            return true;
        double xCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getY();
        if ((Math.abs(m_posList.get(current).getX()) + PIDValues.X_THRESHOLD) > xCurrent &&
                (Math.abs(m_posList.get(current).getX()) - PIDValues.X_THRESHOLD) < xCurrent &&
                (Math.abs(m_posList.get(current).getY()) + PIDValues.Y_THRESHOLD) > yCurrent &&
                (Math.abs(m_posList.get(current).getY()) - PIDValues.Y_THRESHOLD) < yCurrent) {
            current++;
            if (m_posList.size() == current)
                return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).stop();
    }

}
