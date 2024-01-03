package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

import com.kauailabs.navx.frc.AHRS;

public class Swerve extends SubsystemBase {

    //array of swerve modules
    public SwerveModule[] m_modules;

    //robot gyro
    private AHRS m_gyro;

    //Swerve instance
    private static Swerve m_instance;

    private double m_headingTargetAngle;
    private double m_rotationSpeed;
    private PIDController m_headingPidController;
    private double m_x, m_y;

    /**
     * @param usesAbsEncoder -if robot got can coders connected to the steering motors
     */
    public Swerve(boolean usesAbsEncoder) {
        m_modules = new SwerveModule[Consts.physicalMoudulesVector.length];
        //create modules array
        if (usesAbsEncoder) {
            m_modules[0] = new SwerveModule(Consts.TOP_RIGHT_DRIVE_PORT, Consts.TOP_RIGHT_STEERING_PORT,
                    Consts.TOP_RIGHT_CANCODER, Consts.TOP_RIGHT_CANCODER_OFFSET);
            m_modules[1] = new SwerveModule(Consts.TOP_LEFT_DRIVE_PORT, Consts.TOP_LEFT_STEERING_PORT,
                    Consts.TOP_LEFT_CANCODER, Consts.TOP_LEFT_CANCODER_OFFSET);
            m_modules[2] = new SwerveModule(Consts.DOWN_RIGHT_DRIVE_PORT, Consts.DOWN_RIGHT_STEERING_PORT,
                    Consts.DOWN_RIGHT_CANCODER, Consts.DOWN_RIGHT_CANCODER_OFFSET);
            m_modules[3] = new SwerveModule(Consts.DOWN_LEFT_DRIVE_PORT, Consts.DOWN_LEFT_STEERING_PORT,
                    Consts.DOWN_LEFT_CANCODER, Consts.DOWN_LEFT_CANCODER_OFFSET);
        } else {
            m_modules[0] = new SwerveModule(Consts.TOP_RIGHT_DRIVE_PORT, Consts.TOP_RIGHT_STEERING_PORT);
            m_modules[1] = new SwerveModule(Consts.TOP_LEFT_DRIVE_PORT, Consts.TOP_LEFT_STEERING_PORT);
            m_modules[2] = new SwerveModule(Consts.DOWN_RIGHT_DRIVE_PORT, Consts.DOWN_RIGHT_STEERING_PORT);
            m_modules[3] = new SwerveModule(Consts.DOWN_LEFT_DRIVE_PORT, Consts.DOWN_LEFT_STEERING_PORT);
        }
        m_instance = null;
        m_headingPidController = new PIDController(Consts.HEADING_KP, Consts.HEADING_KI, Consts.HEADING_KD);
        m_gyro = new AHRS(SerialPort.Port.kMXP);
        m_headingTargetAngle = m_gyro.getAngle();
        m_headingPidController.setTolerance(Consts.HEADING_TOLERANCE);
        m_rotationSpeed = 0;
        m_x = 0;
        m_y = 0;
        
    }

     /** 
     * @param usesAbsEncoder -if robot got can coders connected to the steering motors
     */
    public static Swerve getInstance(boolean usesAbsEncoder) {
        if (m_instance == null) {
            m_instance = new Swerve(usesAbsEncoder);
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        double currentAngle = m_gyro.getAngle();
        m_headingPidController.setPID(Consts.HEADING_KP, Consts.HEADING_KI, Consts.HEADING_KD); //remove later
        double closestAngle = Consts.closestAngle(currentAngle, m_headingTargetAngle);
        double optimizedAngle = currentAngle + closestAngle;
        m_rotationSpeed = MathUtil.clamp(m_headingPidController.calculate(currentAngle, optimizedAngle), -Consts.MAX_ANGULAR_SPEED.get(), Consts.MAX_ANGULAR_SPEED.get());
 
        odometry();
        SmartDashboard.putNumber("rotationSpeed", m_rotationSpeed);
        SmartDashboard.putNumber("optimizedAngle", optimizedAngle);
        SmartDashboard.putNumber("currentAngle", currentAngle);
        SmartDashboard.putNumber("m_headingTargetAngle", m_headingTargetAngle);

    }

    /**
     * see math on pdf document for more information 
     * @param directionVec - 2d vector that represents target velocity vector (x and y values are between 1 and -1)
     * @param isFieldOriented - true if you want the robot to drive accoring to field coordinates false if you want the robot to drive accoring to robot facing direction
     */
    public void drive(Vector2d driveVec, boolean isFieldOriented) {
            //if drive values are 0 stop moving
            if(driveVec.mag() == 0 && m_rotationSpeed == 0){
                for(int i = 0; i < m_modules.length; i++){
                    m_modules[i].setVelocity(0);
                }
            }
            
            //convert drive vector to m/s
            driveVec.mul(Consts.MAX_SPEED.get());

            //convert driveVector to field oriented
            if(isFieldOriented){
                driveVec.rotate(Math.toRadians(m_gyro.getYaw()));
            }
            
            //calculate rotation vectors
            Vector2d[] rotVecs = new Vector2d[m_modules.length];
            for(int i = 0; i < rotVecs.length; i++){
                rotVecs[i] = new Vector2d(Consts.physicalMoudulesVector[i]);
                rotVecs[i].rotate(Math.toRadians(90));
                //change magnitude of rot vector to m_rotationSpeed
                rotVecs[i].normalise();
                rotVecs[i].mul(m_rotationSpeed);
            }
            

            Vector2d[] sumVectors = new Vector2d[m_modules.length];
            for(int i = 0; i < sumVectors.length; i++){
                //sum rot and drive vectors 
                sumVectors[i] = new Vector2d(driveVec);
                sumVectors[i].add(rotVecs[i]);
                
                //set module state             
                m_modules[i].setState(sumVectors[i]);
            }
    }
    
    public void rotateBy(double angle){
        m_headingTargetAngle += angle;
    }

    public void rotateTo(double angle){
        m_headingTargetAngle = angle;
    }

    public void zeroYaw(){
        m_gyro.zeroYaw();
        m_headingTargetAngle = m_gyro.getAngle();
    }

    public void zeroModulesAngles(){
        for(int i = 0; i < m_modules.length; i++){
            m_modules[i].setModuleAngle(0);
        }
    }

    /**
     * put the current position of every can coder in the every rotation motor's integrated encoder
     * activate this at the start
     */
    public void setModulesToAbs(){
        for(int i = 0; i < m_modules.length; i++){
            m_modules[i].setModulesToAbs();
        }
    }
    

    /**
     * set the speeds of all motors to 0
     */
    public void stop() {
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].turnOff();
        }
    }
    public Gyro getGyro(){
        return m_gyro;
    }
    public void resetOdometry(){
        for(int i = 0 ; i < m_modules.length; i++){
            m_modules[i].updatePos(0);
        }
        m_x = 0;
        m_y = 0;
    }

    public SwerveModule getModule(int idx){
        return m_modules[idx];
    }

    public double getX(){
        return m_x;
    }

    public double getY(){
        return m_y;
    }

    public void odometry(){
        double deltaX = 0, deltaY = 0;
        for(int i = 0; i < m_modules.length; i++){
            double deltaP = m_modules[i].getPos() - m_modules[i].m_currentPosition;
            deltaX = (Math.cos(Math.toRadians(m_modules[i].getAngle())) * deltaP) / 4;
            deltaY = (Math.sin(Math.toRadians(m_modules[i].getAngle())) * deltaP) / 4;
            Vector2d tempVec = new Vector2d(deltaX, deltaY);
            tempVec.rotate(-Math.toRadians(m_gyro.getYaw()));
            m_x += tempVec.x;
            m_y += tempVec.y;
            m_modules[i].updatePos();
            SmartDashboard.putNumber("deltaX", deltaX);
            SmartDashboard.putNumber("deltaY", deltaY);
        }
        SmartDashboard.putNumber("x",m_x);
        SmartDashboard.putNumber("y", m_y);
    }
}
    