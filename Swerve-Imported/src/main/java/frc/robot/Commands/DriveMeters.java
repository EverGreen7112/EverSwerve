package frc.robot.Commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utils.SwervePoint;

public class DriveMeters extends CommandBase{

    private double m_distance; // distance to drive in a straight line of the robot
    private double m_angle; // current angle of the robot
    private double m_currentX, m_currentY;

    public DriveMeters(double distance, double angle, double currentX, double currentY){
        m_distance = distance;
        m_angle = angle;
        m_currentX = currentX;
        m_currentY = currentY;
    }

    @Override
    public void initialize() {
        // X value of the distance to drive to in respect to the angle
        double tempX = m_currentX + Math.cos(Math.toRadians(m_angle) * m_distance);
        // Y value of the distance to drive to in respect to the angle
        double tempY = m_currentY + Math.sin(Math.toRadians(m_angle) * m_distance);
        // Making a list to enter the point into
        ArrayList<SwervePoint> posList = new ArrayList<SwervePoint>();
        posList.add(new SwervePoint(tempX, tempY, m_angle));
        // Activating the FollowRoute function to go to point
        CommandScheduler.getInstance().schedule(new FollowRoute(posList));
    }

    @Override
    public void execute() {
        
    }
    
}
