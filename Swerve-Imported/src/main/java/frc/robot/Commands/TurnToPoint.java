package frc.robot.Commands;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Vector2d;

public class TurnToPoint extends CommandBase implements Constants{
    //target poiny
    private Vector2d m_target;

    public TurnToPoint(Vector2d target){
        m_target = target;
    }

    public TurnToPoint(double targetX, double targetY){
        m_target = new Vector2d(targetX, targetY);
    }

    @Override
    public void execute() {
        //get current position of robot
        Vector2d currentPos = Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).getPos();
        //get the vector between current and target vectors
        Vector2d deltaPos = new Vector2d(m_target.x - currentPos.x, m_target.y - currentPos.y ); 
        double targetAngle = 90 - Math.toDegrees(deltaPos.theta());
        SmartDashboard.putNumber("target angle", targetAngle);
        //rotate swerve to point
        Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).rotateTo(targetAngle);
    }
}
