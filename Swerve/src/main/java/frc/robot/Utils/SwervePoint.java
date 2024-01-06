package frc.robot.Utils;

public class SwervePoint {
    private double m_x;
    private double m_y;
    private double m_angle;
    
    public SwervePoint(double x, double y, double angle){
        this.m_x = x;
        this.m_y = y;
        this.m_angle = angle;
    }

    public double getX() {
        return m_x;
    }

    public void setX(double x) {
        this.m_x = x;
    }

    public double getY() {
        return m_y;
    }

    public void setY(double y) {
        this.m_y = y;
    }

    public double getAngle() {
        return m_angle;
    }

    public void setAngle(double angle) {
        this.m_angle = angle;
    }

    
}
