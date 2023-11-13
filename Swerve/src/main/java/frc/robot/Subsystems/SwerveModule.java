package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class SwerveModule extends SubsystemBase {

    //motor controller that controls the speed of the modules 
    private CANSparkMax m_speedMotor;

    //motor controller that controls the rotation of the modules
    private CANSparkMax m_rotationMotor;

    //can coder to save the absolute position of the module
    private CANCoder m_coder;
    
    /**
     * 
     * port = id(team convention)
     * @param speedPort - speed motor id of module
     * @param rotationPort - rotation motor id of module
     */
    public SwerveModule(int speedPort, int rotationPort) {
        m_speedMotor = new CANSparkMax(speedPort, MotorType.kBrushless);
        m_rotationMotor = new CANSparkMax(rotationPort, MotorType.kBrushless);

        //restore the deafult values of motor so nothing would interrupt motor
        m_rotationMotor.restoreFactoryDefaults();
        
        //set idle mode of rotation motor
        m_rotationMotor.setIdleMode(IdleMode.kCoast);
        
        //config pid values
        m_rotationMotor.getPIDController().setP(Consts.WHEEL_ROTATION_KP);
        m_rotationMotor.getPIDController().setI(Consts.WHEEL_ROTATION_KI);
        m_rotationMotor.getPIDController().setD(Consts.WHEEL_ROTATION_KD);
    }

    /**
     * port = id(7112 team convention)
     * @param speedPort - speed motor id of module
     * @param rotationPort - rotation motor id of module
     * @param absoluteEncoderPort - can coder id of module
     * @param canCoderOffset - can coder magnet offset offset
     */
    public SwerveModule(int speedPort, int rotationPort, int absoluteEncoderPort, double canCoderOffset) {
        this(speedPort, rotationPort);
        
        //configure cancoder
        m_coder = new CANCoder(absoluteEncoderPort);
        m_coder.configFactoryDefault();
        m_coder.configMagnetOffset(canCoderOffset,50);
        
        //convert rotation motor position value to degrees and take care of gear ratio
        m_rotationMotor.getEncoder().setPositionConversionFactor(Consts.ROTATION_GEAR_RATIO * 360); //degrees and gear ratio

        //take care of speed motor velocity gear velocity
        m_speedMotor.getEncoder().setVelocityConversionFactor(Consts.DRIVE_GEAR_RATIO);
    }

    /**
     * set the speeds of motors to 0
     */
    public void turnOff() {
        this.m_rotationMotor.set(0);
        this.m_speedMotor.set(0);
    }

    /**
     * put the current position of the can coder in the rotation motor's integrated encoder
     */
    public void initModulesToAbs(){
        m_rotationMotor.getEncoder().setPosition(m_coder.getAbsolutePosition());
    }

    public double getCoderPos(){
        return m_coder.getPosition();
    }

    public double getPos(){
        return m_rotationMotor.getEncoder().getPosition();
    }

    public void setState(double speed, double angle){
        setState(new Vector2d(speed * Math.cos(Math.toRadians(angle)), speed * Math.sin(Math.toRadians(angle))));
    }

    /**
     * 
     * @param desiredState - 2d vector - magnitude represents the target speed (-1 - 1)  
     *                                   angle represents the target angle 
     */
    public void setState(Vector2d desiredState) {
        double targetAngle = Math.toDegrees(desiredState.theta()); //convert target angle from radians to degrees
        double targetSpeed = desiredState.mag(); //get target speed

        double optimizedTargetAngle = getPos() + Consts.closestAngle(getPos(), targetAngle);
        //turn module to target angle
        m_rotationMotor.getPIDController().setReference(optimizedTargetAngle, ControlType.kPosition);

        //set speed of module at target speed
        m_speedMotor.set(targetSpeed);
    }

    /**
     * turn module to targetAngle
     * @param targetAngle in degrees
     */
    public void turnToAngle(double targetAngle){
        m_rotationMotor.getPIDController().setReference(targetAngle, ControlType.kPosition);
    }

}
