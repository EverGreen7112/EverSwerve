package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;
import com.kauailabs.navx.frc.AHRS;

public class Swerve extends SubsystemBase {

    //array of swerve modules
    public SwerveModule[] m_modules = new SwerveModule[Consts.physicalMoudulesVector.length];

    //robot gyro
    private AHRS m_gyro;

    //Swerve instance
    private static Swerve m_instance = null;

    private double m_headingTargetAngle;
    private double m_rotationSpeed;
    private double m_currentTime;
    private PIDController m_headingPidController = new PIDController(Consts.HEADING_KP.get(), Consts.HEADING_KI, Consts.HEADING_KD.get());
    private double x, y;

    /**
     * 
     * @param usesAbsEncoder -if robot got can coders connected to the steering motors
     */
    public Swerve(boolean usesAbsEncoder) {
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
        m_gyro = new AHRS(SerialPort.Port.kMXP);
        m_headingTargetAngle = m_gyro.getAngle();
        
        m_currentTime = System.currentTimeMillis() / 1000.0;
        x = 0;
        y = 0;
    }

     /**
     * 
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
        m_headingPidController.setPID(Consts.HEADING_KP.get(), Consts.HEADING_KI, Consts.HEADING_KD.get()); //remove later
        double closestAngle = Consts.closestAngle(currentAngle, m_headingTargetAngle);
        double optimizedAngle = currentAngle + closestAngle;
        m_rotationSpeed = -MathUtil.clamp(m_headingPidController.calculate(currentAngle, optimizedAngle), -Consts.MAX_ANGULAR_SPEED.get(), Consts.MAX_ANGULAR_SPEED.get());
        if(Math.abs(closestAngle) < Consts.HEADING_TOLERANCE){
            m_rotationSpeed = 0.001;
        } 
        SmartDashboard.putNumber("rotationSpeed", m_rotationSpeed);

        double deltaTime = (System.currentTimeMillis() / 1000.0) - m_currentTime;
        odometry(deltaTime);

        m_currentTime = System.currentTimeMillis() / 1000.0;
    }

    /**
     * see math on pdf document for more information 
     * @param directionVec - 2d vector that represents target velocity vector (x and y values are between 1 and -1)
     * @param isFieldOriented - true if you want the robot to drive accoring to field coordinates false if you want the robot to drive accoring to robot facing direction
     */
    public void drive(Vector2d directionVec, boolean isFieldOriented) {
        Vector2d dirVec = directionVec;

        if (isFieldOriented) {
            // rotates the direction vector to fit field coordinate system
            dirVec = dirVec.rotate(Math.toRadians(m_gyro.getYaw()));
        }

        Vector2d[] rotVecs = new Vector2d[m_modules.length];

        for (int i = 0; i < rotVecs.length; i++) {
            rotVecs[i] = new Vector2d(Consts.physicalMoudulesVector[i]);
            rotVecs[i].rotate(Math.toRadians(0));
            // rotVecs[i].rotate(Math.toRadians(90));
        }

        // we need the max magnitude and because all of the magnitudes are equal there is no reason to search for the biggest one
        double mag = rotVecs[0].mag();

        // normalize by the vector with the biggest magnitude
        for (int i = 0; i < rotVecs.length; i++) {
            rotVecs[i].mul(1 / mag); // normalize
            rotVecs[i].mul(m_rotationSpeed); // mul by the rotation speed
        }

        // add vectors
        Vector2d[] finalVecs = new Vector2d[m_modules.length];
        for (int i = 0; i < finalVecs.length; i++) {
            finalVecs[i] = new Vector2d(rotVecs[i]);
            finalVecs[i].add(dirVec);
        }

        // find max magnitude
        mag = finalVecs[0].mag();
        for (int i = 1; i < finalVecs.length; i++) {
            mag = Math.max(mag, finalVecs[i].mag());
        }

        // normalize by the vector with the biggest magnitude
        for (int i = 0; i < finalVecs.length; i++) {
            finalVecs[i].mul(1 / mag); // divide by maxMagnitude
        }

        mag = Math.min(Consts.SPEED.get() * mag, Consts.MAX_SPEED.get());
        // set target state of module
        for (int i = 0; i < finalVecs.length; i++) {
            finalVecs[i].mul(mag);
            m_modules[i].setState(finalVecs[i]);
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
    public void initModulesToAbs(){
        for(int i = 0; i < m_modules.length; i++){
            m_modules[i].initModulesToAbs();
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

    public SwerveModule getModule(int idx){
        return m_modules[idx];
    }
    public void odometry(double deltaTime){
        double vX = 0, vY = 0;
        for(int i = 0; i < m_modules.length; i++){
            double tempX = Math.cos(Math.toRadians(m_modules[i].getAngle())) * m_modules[i].getSpeed();
            double tempY = Math.sin(Math.toRadians(m_modules[i].getAngle())) * m_modules[i].getSpeed();
            Vector2d tempVector = new Vector2d(tempX,tempY);
            tempVector.rotate(Math.toRadians(m_gyro.getAngle()));
            vX += tempVector.x;
            vY += (tempVector.y) * 0.206;
        }
        
        x += vX * deltaTime;
        y += vY * deltaTime;

        SmartDashboard.putNumber("deltaTime",deltaTime);
        SmartDashboard.putNumber("vX", vX);
        SmartDashboard.putNumber("vY", vY);
        SmartDashboard.putNumber("x",x);
        SmartDashboard.putNumber("y", y);
    }
}
    