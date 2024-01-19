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
    private ArrayList<SwervePoint> m_posList;
    private int current;

    public FollowRoute(ArrayList<SwervePoint> posList) {
        m_posList = posList;
    }

    @Override
    public void initialize() {
        m_xPidController = new PIDController(PIDValues.POS_KP, PIDValues.POS_KI, PIDValues.POS_KD);
        m_yPidController = new PIDController(PIDValues.POS_KP, PIDValues.POS_KI, PIDValues.POS_KD);
        current = 0;
    }

    @Override
    public void execute() {
        if (m_posList.isEmpty())
            return;
        // get current values
        double xCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getY();
        // calculate outputs
        double xOutput = MathUtil.clamp(m_xPidController.calculate(m_posList.get(current).getX() - xCurrent), -1, 1);
        double yOutput = MathUtil.clamp(m_yPidController.calculate(m_posList.get(current).getY() - yCurrent), -1, 1);

        // apply outputs
        Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).drive(new Vector2d(xOutput, yOutput), true);
        Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).rotateTo(m_posList.get(current).getAngle());
    }

    @Override
    public boolean isFinished() {
        if (m_posList.isEmpty())
            return true;
        //get current state of robot
        double xCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getY();
        double headingCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getGyro().getAngle();
        if ((   //check if entered x threshold
                (Math.abs(m_posList.get(current).getX()) + PIDValues.X_TOLERANCE) > Math.abs(xCurrent) &&
                (Math.abs(m_posList.get(current).getX()) - PIDValues.X_TOLERANCE) < Math.abs(xCurrent) &&
                //check if entered y threshold
                (Math.abs(m_posList.get(current).getY()) + PIDValues.Y_TOLERANCE) > Math.abs(yCurrent) &&
                (Math.abs(m_posList.get(current).getY()) - PIDValues.Y_TOLERANCE) < Math.abs(yCurrent) &&
                //check if entered heading threshold
                (Math.abs(m_posList.get(current).getAngle()) + PIDValues.HEADING_TOLERANCE) > Math.abs(headingCurrent) &&
                (Math.abs(m_posList.get(current).getAngle()) - PIDValues.HEADING_TOLERANCE) < Math.abs(headingCurrent)   
            )) {
            current++;
        }
        return current >= m_posList.size();
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).stop();
        current = 0;
    }

}
