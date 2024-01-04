package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class SwerveModule extends SubsystemBase {

    //motor controller that controls the speed of the modules 
    public CANSparkMax m_driveMotor;

    //motor controller that controls the rotation of the modules
    private CANSparkMax m_steeringMotor;

    //can coder to save the absolute position of the module
    private CANCoder m_coder;

    public double m_currentPosition;

    
    /**
     * 
     * port = id(team convention)
     * @param drivePort - speed motor id of module
     * @param steeringPort - rotation motor id of module
     */
    public SwerveModule(int drivePort, int steeringPort) {
        m_driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
        m_steeringMotor = new CANSparkMax(steeringPort, MotorType.kBrushless);

        //restore the deafult values of motor so nothing would interrupt motor
        m_steeringMotor.restoreFactoryDefaults();
        m_driveMotor.restoreFactoryDefaults();
        
        //set idle mode of rotation motor
        m_steeringMotor.setIdleMode(IdleMode.kCoast);
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        
        //config angle pid values
        m_steeringMotor.getPIDController().setP(Consts.WHEEL_ANGLE_KP);
        m_steeringMotor.getPIDController().setI(Consts.WHEEL_ANGLE_KI);
        m_steeringMotor.getPIDController().setD(Consts.WHEEL_ANGLE_KD);

        //config velocity pid values
        m_driveMotor.getPIDController().setP(Consts.WHEEL_VELOCITY_KP);
        m_driveMotor.getPIDController().setI(Consts.WHEEL_VELOCITY_KI);
        m_driveMotor.getPIDController().setD(Consts.WHEEL_VELOCITY_KD);
        m_driveMotor.getPIDController().setFF(Consts.WHEEL_VELOCITY_KF);

        //convert rotation motor position value to degrees and take care of gear ratio
        m_steeringMotor.getEncoder().setPositionConversionFactor(Consts.STEERING_GEAR_RATIO * 360); //degrees and gear ratio

        //take care of speed motor velocity gear velocity
        m_driveMotor.getEncoder().setVelocityConversionFactor(Consts.DRIVE_GEAR_RATIO * Consts.WHEEL_PERIMETER / 60.0); //convert from rpm to m/s
        //turn position to meters
        m_driveMotor.getEncoder().setPositionConversionFactor(Consts.DRIVE_GEAR_RATIO * Consts.WHEEL_PERIMETER);

        m_driveMotor.getEncoder().setPosition(0);
        m_currentPosition = m_driveMotor.getEncoder().getPosition();
    }

    /**
     * port = id(7112 team convention)
     * @param drivePort - speed motor id of module
     * @param steeringPort - rotation motor id of module
     * @param absoluteEncoderPort - can coder id of module
     * @param canCoderOffset - can coder magnet offset offset
     */
    public SwerveModule(int drivePort, int steeringPort, int absoluteEncoderPort, double canCoderOffset) {
        this(drivePort, steeringPort);
        
        //configure cancoder
        m_coder = new CANCoder(absoluteEncoderPort);
        m_coder.configFactoryDefault();
        m_coder.configMagnetOffset(canCoderOffset,50);
    }

    @Override
    public void periodic() {}

    /**
     * set the speeds of motors to 0
     */
    public void turnOff() {
        this.m_steeringMotor.set(0);
        this.m_driveMotor.set(0);
    }

    /**
     * put the current position of the can coder in the rotation motor's integrated encoder
     */
    public void setModulesToAbs(){
        m_steeringMotor.getEncoder().setPosition(m_coder.getAbsolutePosition());
    }

    public void setModuleAngle(double angle){
        m_steeringMotor.getEncoder().setPosition(angle);
    }

    public double getCoderPos(){
        return m_coder.getAbsolutePosition();
    }

    public double getAngle(){
        return m_steeringMotor.getEncoder().getPosition();
    }

    /**
     * 
     * @return current module velocity in m/s
     */
    public double getVelocity(){
        return m_driveMotor.getEncoder().getVelocity();
    }

    /**
     * @param velocity - target module velocity in m/s
     */
    public void setVelocity(double velocity){
        m_driveMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
    }

    public void setState(double speed, double angle){
        setState(new Vector2d(speed * Math.cos(Math.toRadians(angle)), speed * Math.sin(Math.toRadians(angle))));
    }

    /**
     * 
     * @param desiredState - 2d vector - magnitude represents the target speed  
     *                                   angle represents the target angle 
     */
    public void setState(Vector2d desiredState) {        
       //rotate vector by 90 because we want 0 degrees to be in the front and not in the right
       desiredState.rotate(Math.toRadians(90));

       //get polar values from desired state vector
       double targetSpeed = desiredState.mag(); //get target speed
       double targetAngle = Math.toDegrees(desiredState.theta()); //convert target angle from radians to degrees
       
       double currentAngle = getAngle();

        //calculate optimal delta angle
       double optimizedFlippedDeltaTargetAngle = Consts.closestAngle(currentAngle, targetAngle - 180);
       double optimizedNormalDeltaTargetAngle = Consts.closestAngle(currentAngle, targetAngle);

       double optimizedDeltaTargetAngle = 0;
        if(Math.abs(optimizedNormalDeltaTargetAngle) > Math.abs(optimizedFlippedDeltaTargetAngle)){
            optimizedDeltaTargetAngle = optimizedFlippedDeltaTargetAngle;
        }
        else{
            optimizedDeltaTargetAngle = optimizedNormalDeltaTargetAngle;
        }

       //turn module to target angle
       m_steeringMotor.getPIDController().setReference(currentAngle + optimizedDeltaTargetAngle, ControlType.kPosition);

       //dot product to current state
       targetSpeed *= Math.cos(Math.toRadians(optimizedNormalDeltaTargetAngle));
    
       //set speed of module at target speed
       m_driveMotor.getPIDController().setReference(targetSpeed, ControlType.kVelocity);
   }


    /**
     * turn module to targetAngle
     * @param targetAngle in degrees
     */

     public void turnToAngle(double targetAngle){
        m_steeringMotor.getPIDController().setReference(targetAngle, ControlType.kPosition);
    }
    public double getPos(){
        return m_driveMotor.getEncoder().getPosition();
    }

    public void updatePos(){
        m_currentPosition = this.getPos();
    }

    public void updatePos(double pos){
        m_driveMotor.getEncoder().setPosition(pos);
    }
}
