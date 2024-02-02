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
        addRequirements(Swerve.getInstance(SwerveValues.USES_ABS_ENCODER));
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
        double xOutput = MathUtil.clamp(m_xPidController.calculate(xCurrent, m_posList.get(current).getX()), -Constants.SpeedValues.MAX_SPEED.get(),
         Constants.SpeedValues.MAX_SPEED.get());
        double yOutput = MathUtil.clamp(m_yPidController.calculate(yCurrent, m_posList.get(current).getY()), -Constants.SpeedValues.MAX_SPEED.get(),
         Constants.SpeedValues.MAX_SPEED.get());

        // apply outputs
        Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).driveFieldOrientedAngle(new Vector2d(xOutput, yOutput));
        Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).turnToFieldOriented(m_posList.get(current).getAngle());
    }

    @Override
    public boolean isFinished() {
        if (m_posList.isEmpty())
            return true;
        //get current state of robot
        double xCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getY();
        double headingCurrent = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getFieldOrientedAngle();
        if ((   //check if entered x threshold
                Math.abs(m_posList.get(current).getX() - xCurrent) < PIDValues.X_TOLERANCE &&
                //check if entered y threshold
                Math.abs(m_posList.get(current).getY() - yCurrent) < PIDValues.Y_TOLERANCE &&
                //check if entered heading threshold
                (Math.abs(m_posList.get(current).getAngle() - headingCurrent) % 360) < PIDValues.HEADING_TOLERANCE
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
