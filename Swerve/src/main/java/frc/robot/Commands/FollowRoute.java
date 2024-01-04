package frc.robot.Commands;

import java.util.ArrayList;

import org.opencv.core.Point;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class FollowRoute extends CommandBase {
    private PIDController m_xPidController;
    private PIDController m_yPidController;
    private double m_targetAngle;
    private ArrayList<Point> m_posList;
    private int current;

    public FollowRoute(ArrayList<Point> posList, double targetAngle) {
        m_posList = posList;
        m_targetAngle = targetAngle;
        m_xPidController = new PIDController(Consts.X_KP, Consts.X_KI, Consts.X_KD);
        m_yPidController = new PIDController(Consts.Y_KP, Consts.Y_KI, Consts.Y_KD);
    }

    @Override
    public void initialize() {
        current = 0;
    }

    @Override
    public void execute() {
        if (m_posList.isEmpty())
            return;
        //get current position
        double xCurrent = Swerve.getInstance(Consts.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(Consts.USES_ABS_ENCODER).getY();
        //calculate outputs
        double xOutput = MathUtil.clamp(m_xPidController.calculate(xCurrent, m_posList.get(current).x), -1, 1);
        double yOutput = MathUtil.clamp(m_yPidController.calculate(yCurrent, m_posList.get(current).y), -1, 1);
        //round values
        xCurrent = Consts.roundAfterDecimalPoint(xCurrent, 2);
        yCurrent = Consts.roundAfterDecimalPoint(yCurrent, 2);
        //apply outputs
        Swerve.getInstance(Consts.USES_ABS_ENCODER).drive(new Vector2d(xOutput, yOutput), false);
        Swerve.getInstance(Consts.USES_ABS_ENCODER).rotateTo(m_targetAngle);
    }

    @Override
    public boolean isFinished() {
        if (m_posList.isEmpty())
            return true;
        double xCurrent = Swerve.getInstance(Consts.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(Consts.USES_ABS_ENCODER).getY();
        if ((Math.abs(m_posList.get(current).x) + Consts.X_THRESHOLD) > xCurrent &&
                (Math.abs(m_posList.get(current).x) - Consts.X_THRESHOLD) < xCurrent &&
                (Math.abs(m_posList.get(current).y) + Consts.Y_THRESHOLD) > yCurrent &&
                (Math.abs(m_posList.get(current).y) - Consts.Y_THRESHOLD) < yCurrent) {
            current++;
            if (m_posList.size() == current)
                return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance(Consts.USES_ABS_ENCODER).stop();
    }

}
