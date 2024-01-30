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
     * see math on pdf document for more information
     * 
     * @param directionVec    - 2d vector that represents target velocity vector (x
     *                        and y values are between 1 and -1)
     * @param isFieldOriented - true if you want the robot to drive accoring to
     *                        field coordinates false if you want the robot to drive
     *                        accoring to robot facing direction
     */
    public void drive(Vector2d driveVec, boolean isFieldOriented) {
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
     * rotate chassis by angle
     */
    public void rotateBy(double angle) {
        m_headingTargetAngle += angle;
    }

    /**
     * 
     * rotate chassis to angle
     */
    public void rotateTo(double angle) {
        m_headingTargetAngle = angle;
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
        m_robotHeadingFromVision = -robotHeadingFromVision;
        m_angleOffset = robotHeadingFromVision - m_gyro.getYaw();
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
     * @return
     */
    public Vector2d getPos(){
        return new Vector2d(getX(), getY());
    }

    public double getAngleWithOffset(){
        return -(m_gyro.getAngle() + m_angleOffset);
    }

    public void odometry() {
        double deltaX = 0, deltaY = 0;
        for (int i = 0; i < m_modules.length; i++) {
            double deltaP = m_modules[i].getPos() - m_modules[i].m_currentPosition;
            deltaX = (Math.cos(Math.toRadians(m_modules[i].getAngle())) * deltaP) / 4;
            deltaY = (Math.sin(Math.toRadians(m_modules[i].getAngle())) * deltaP) / 4;
            Vector2d deltaVec = new Vector2d(deltaX, deltaY);
            //rotate by yaw to get the values as field oriented
            // -90 and -angle to convert values to the rights axises

            deltaVec.rotate(Math.toRadians(this.getAngleWithOffset()));
            m_x += deltaVec.x;
            m_y += deltaVec.y;
            m_modules[i].updatePos();
        }
        SmartDashboard.putNumber("x", m_x);
        SmartDashboard.putNumber("y", m_y);
        SmartDashboard.putNumber("heading vision", m_angleOffset);
    }
}
