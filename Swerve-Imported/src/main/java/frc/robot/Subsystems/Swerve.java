package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Funcs;
import frc.robot.Utils.Vector2d;

public class Swerve extends SubsystemBase implements Constants {

    // array of swerve modules
    private SwerveModule[] m_modules;

    // robot gyro
    private AHRS m_gyro;

    // Swerve instance
    private static Swerve m_instance;
    // heading pid target angle
    private double m_headingTargetAngle;
    // current rotation speed
    private double m_rotationSpeed;
    // heading pid controller
    private PIDController m_headingPidController;
    // current x and y positions
    private double m_x, m_y;
    // robot heading angle from the localization vision 
    private double m_robotHeadingFromVision;
    // angle offset of robot
    private double m_angleOffset = 0;

    /**
     * @param usesAbsEncoder -if robot got can coders connected to the steering
     *                       motors
     */
    public Swerve(boolean usesAbsEncoder) {
        m_modules = new SwerveModule[SwerveValues.physicalMoudulesVector.length];
        // create modules array
        if (usesAbsEncoder) {
            m_modules[0] = new SwerveModule(SwerveValues.TOP_RIGHT_DRIVE_PORT, SwerveValues.TOP_RIGHT_STEERING_PORT,
                    SwerveValues.TOP_RIGHT_CANCODER, SwerveValues.TOP_RIGHT_CANCODER_OFFSET);
            m_modules[1] = new SwerveModule(SwerveValues.TOP_LEFT_DRIVE_PORT, SwerveValues.TOP_LEFT_STEERING_PORT,
                    SwerveValues.TOP_LEFT_CANCODER, SwerveValues.TOP_LEFT_CANCODER_OFFSET);
            m_modules[2] = new SwerveModule(SwerveValues.DOWN_RIGHT_DRIVE_PORT, SwerveValues.DOWN_RIGHT_STEERING_PORT,
                    SwerveValues.DOWN_RIGHT_CANCODER, SwerveValues.DOWN_RIGHT_CANCODER_OFFSET);
            m_modules[3] = new SwerveModule(SwerveValues.DOWN_LEFT_DRIVE_PORT, SwerveValues.DOWN_LEFT_STEERING_PORT,
                    SwerveValues.DOWN_LEFT_CANCODER, SwerveValues.DOWN_LEFT_CANCODER_OFFSET);
        } else {
            m_modules[0] = new SwerveModule(SwerveValues.TOP_RIGHT_DRIVE_PORT, SwerveValues.TOP_RIGHT_STEERING_PORT);
            m_modules[1] = new SwerveModule(SwerveValues.TOP_LEFT_DRIVE_PORT, SwerveValues.TOP_LEFT_STEERING_PORT);
            m_modules[2] = new SwerveModule(SwerveValues.DOWN_RIGHT_DRIVE_PORT, SwerveValues.DOWN_RIGHT_STEERING_PORT);
            m_modules[3] = new SwerveModule(SwerveValues.DOWN_LEFT_DRIVE_PORT, SwerveValues.DOWN_LEFT_STEERING_PORT);
        }
        m_instance = null;
        m_headingPidController = new PIDController(PIDValues.HEADING_KP, PIDValues.HEADING_KI, PIDValues.HEADING_KD);
        m_gyro = new AHRS(SerialPort.Port.kMXP);
        m_headingTargetAngle = m_gyro.getAngle();
        m_headingPidController.setTolerance(PIDValues.HEADING_TOLERANCE);
        m_rotationSpeed = 0;
        m_x = 0;
        m_y = 0;
        m_robotHeadingFromVision = 0;
    }

    /**
     * @param usesAbsEncoder -if robot got can coders connected to the steering
     *                       motors
     */
    public static Swerve getInstance(boolean usesAbsEncoder) {
        if (m_instance == null) {
            m_instance = new Swerve(usesAbsEncoder);
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        //convert max angular speed to m/s instead from deg/s
        double ms_max_angular_speed = (SpeedValues.MAX_ANGULAR_SPEED.get() / 360.0) * SwerveValues.ROBOT_BOUNDING_CIRCLE_PERIMETER;
        // get current speed
        double currentAngle = m_gyro.getAngle();
        // calculate optimized target angle
        double closestAngle = Funcs.closestAngle(currentAngle, m_headingTargetAngle);
        double optimizedAngle = currentAngle + closestAngle;
        // get pid output
        m_rotationSpeed = MathUtil.clamp(m_headingPidController.calculate(currentAngle, optimizedAngle),
                -ms_max_angular_speed, ms_max_angular_speed);
        // calculate odometry
        odometry();
    }

    /**
     * this function provides a trully field oriented drive as opposed to {@link #driveRobotOrientedAngle(Vector2d, boolean) driveRobotOrientedAngle}.
     * 
     * 
     * this function uses the fields coordinate system as well as the fields angle of 0 to make sure the same vector
     * would cause you to always drive in the same direction regardless of where the robot was facing on init.
     * 
     * this function is good for autonoumous but is not good for driving with joysticks
     * 
     * @param driveVec - 2d vector that represents target velocity vector (x
     *                   and y values are between 1 and -1), keep in mind, a direction
     *                   of 0 degrees will be the same regardless of where the robot was facing 
     *                   when it was turned on
     */
    public void driveFieldOrientedAngle(Vector2d driveVec) {
        Vector2d robotOrientedDriveVec = driveVec.copy();
        robotOrientedDriveVec.rotateBy(Math.toRadians(-m_angleOffset - 90)); // -90 because setState adds 90
        robotOrientedDriveVec.x *= -1;
        driveRobotOrientedAngle(robotOrientedDriveVec, true);

    }

    /**
     * DO NOT USE THIS FUNCTION ACCIDENTLY INSTEAD OF {@link #driveFieldOrientedAngle(Vector2d) driveFieldOrientedAngle},
     * this function is not trully field oriented as the angle is relative to the robots starting point AND NOT THE FIELDS ACTUAL 0!!!
     * this function is usefull for driving in real time BUT SHOULD NOT BE USED FOR AUTONOMOUS OR ANYTHING OF THIS SORT!!!
     * 
     * 
     * see math on pdf document for more information on the how this function works
     * 
     * @param directionVec    - 2d vector that represents target velocity vector (x
     *                        and y values are between 1 and -1), keep in mind, 
     *                        when using field oriented with this function a direction of 0 
     *                        degrees is where ever the robot was facing when it was turned on 
     * @param isFieldOriented - true if you want the robot to drive accoring to
     *                        field coordinates (BUT STILL WITH A ROBOT ORIENTED ANGLE) 
     *                        false if you want the robot to drive accoring to robot facing direction
     *                        
     */
    public void driveRobotOrientedAngle(Vector2d driveVec, boolean isFieldOriented) {
        // if drive values are 0 stop moving
        if (driveVec.mag() == 0 && m_rotationSpeed == 0) {
            for (int i = 0; i < m_modules.length; i++) {
                m_modules[i].stopModule();
            }
        }

        // convert drive vector to m/s
        driveVec.mul(SpeedValues.MAX_SPEED.get());

        // convert driveVector to field oriented
        if (isFieldOriented) {
            driveVec.rotate(Math.toRadians(m_gyro.getYaw()));
        }

        // calculate rotation vectors
        Vector2d[] rotVecs = new Vector2d[m_modules.length];
        for (int i = 0; i < rotVecs.length; i++) {
            rotVecs[i] = new Vector2d(SwerveValues.physicalMoudulesVector[i]);
            rotVecs[i].rotate(Math.toRadians(90));
            // change magnitude of rot vector to m_rotationSpeed
            rotVecs[i].normalise();
            rotVecs[i].mul(m_rotationSpeed);
        }

        Vector2d[] sumVectors = new Vector2d[m_modules.length];
        for (int i = 0; i < sumVectors.length; i++) {
            // sum rot and drive vectors
            sumVectors[i] = new Vector2d(driveVec);
            sumVectors[i].add(rotVecs[i]);

            // set module state
            m_modules[i].setState(sumVectors[i]);
        }
    }

    /**
     * 
     * turn chassis by angle
     */
    public void turnBy(double angleDegrees) {
        m_headingTargetAngle += angleDegrees;
    }

    /**
     * DO NOT USE THIS FUNCTION ACCIDENTLY, most of the times you should use {@link #turnToFieldOriented(double) turnToFieldOriented}.
     * 
     * this function turns chassis to a robot oriented angle, meaning the 0 angle isnt the real 0 but whatever 
     * direction the robot wal looking at when it was turned on
     */
    public void turnToRobotOriented(double angleDegrees) {
        m_headingTargetAngle = angleDegrees;
    }

    /**
     * 
     * makes the robot turn to a field 
     */
    public void turnToFieldOriented(double angleDegrees) {
        m_headingTargetAngle = angleDegrees - m_angleOffset;
    }


    /**
     * set gyro's yaw value to 0 
     */
    public void zeroYaw() {
        m_gyro.zeroYaw();
        m_headingTargetAngle = m_gyro.getAngle();
    }

    public void zeroModulesAngles() {
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setModuleAngle(0);
        }
    }

    /**
     * put the current position of every can coder in the every rotation motor's
     * integrated encoder
     * activate this at the start
     */
    public void setModulesToAbs() {
        for (int i = 0; i < m_modules.length; i++) {
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

    /**
     * DO NOT USE ACCIDENTLY INSTEAD OF {@link #getFieldOrientedAngle() getFieldOrientedAngle}. 
     * 
     * this method returns the gyro which tells you the robot oriented angle of the robot
     *  AND NOT the field oriented angle!!! most of the times you'd need the field oriented angle, so use this carefully
     * @return
     */
    public AHRS getGyro() {
        return m_gyro;
    }

    public void resetOdometry() {
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].updatePos(0);
        }
        m_x = 0;
        m_y = 0;
    }

    public void setOdometryVals(double x, double y, double robotHeadingFromVision){
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].updatePos(0);
        }
        m_x = x;
        m_y = y;
        m_robotHeadingFromVision = robotHeadingFromVision;
        m_angleOffset = m_robotHeadingFromVision - m_gyro.getAngle();
    }

    /**
     * get SwerveModule at idx
     **/
    public SwerveModule getModule(int idx) {
        return m_modules[idx];
    }

    /**  
     * get current position in the x axis
    */
    public double getX() {
        return m_x;
    }

    /**  
     * get current position in the y axis
    */
     public double getY() {
        return m_y;
    }

    /**
     * 
     * @return get the current position as a vector
     */
    public Vector2d getPos(){
        return new Vector2d(getX(), getY());
    }

    /**
     * @return the angle offset that transforms an angle from robot oriented to field oriented
     */
    public double getOffsetAngle(){ 
        return m_angleOffset;
    }

    /**
     * 
     * @return the angle of the robot with an offset to make it field oriented 
     */
    public double getFieldOrientedAngle(){
        return (m_gyro.getAngle() + m_angleOffset);
    }

    public void odometry() {
        double moduleDeltaX = 0, moduleDeltaY = 0;  // these are robot oriented
        double robotDeltaX = 0;
        double robotDeltaY = 0;
        for (int i = 0; i < m_modules.length; i++) {
            double deltaP = m_modules[i].getPos() - m_modules[i].m_currentPosition;

            // these values are all robot oriented
            moduleDeltaX = (Math.cos(Math.toRadians(m_modules[i].getAngle())) * deltaP);
            moduleDeltaY = (Math.sin(Math.toRadians(m_modules[i].getAngle())) * deltaP);
            // Vector2d deltaVec = new Vector2d(moduleDeltaX, moduleDeltaY);
            //rotate by yaw to get the values as field oriented
            // -90 and -angle to convert values to the rights axises
            robotDeltaY += moduleDeltaY;
            robotDeltaX += moduleDeltaX;
            m_modules[i].updatePos(); 
        }
        // takes the average 
        robotDeltaX *= 0.25;
        robotDeltaY *= 0.25;

        // this is the robot's Delta movement in robot oriented coordinates
        Vector2d robotDelta = new Vector2d(robotDeltaX, robotDeltaY);
        robotDelta.rotateBy(Math.toRadians(getFieldOrientedAngle()));  // changes robotDelta to field oriented

        m_x -= robotDelta.x;
        m_y += robotDelta.y;


        SmartDashboard.putNumber("x", m_x);
        SmartDashboard.putNumber("y", m_y);
        SmartDashboard.putNumber("angle", getFieldOrientedAngle());
        SmartDashboard.putNumber("target angle", m_headingTargetAngle);
        SmartDashboard.putNumber("vision angle", m_robotHeadingFromVision);
        SmartDashboard.putNumber("offset", m_angleOffset);
    }
}
