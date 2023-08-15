package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class SwerveModule{

    private CANSparkMax m_speedMotor;
    private CANSparkMax m_rotationMotor;
    private CANCoder m_coder; 

    public SwerveModule(int speedPort, int rotationPort){
        m_speedMotor = new CANSparkMax(speedPort, MotorType.kBrushless);
        m_rotationMotor = new CANSparkMax(rotationPort, MotorType.kBrushless);
        
        m_rotationMotor.getPIDController().setP(Consts.WHEEL_ROTATION_KP);
        m_rotationMotor.getPIDController().setI(Consts.WHEEL_ROTATION_KI);
        m_rotationMotor.getPIDController().setD(Consts.WHEEL_ROTATION_KD);   
    }

    //used for putting starting values from the absolute encoders in the encoders of the sparkmax
    public SwerveModule(int speedPort, int rotationPort, int absoluteEncoderPort){
        this(speedPort, rotationPort);
        m_coder = new CANCoder(absoluteEncoderPort);
    }

    public void setAbsPosInSpark(){
        m_rotationMotor.getEncoder().setPosition(m_coder.getPosition() / 360); 
    }


    public Vector2d getState(){
        double currentAngle = Consts.rotationsToDegrees(m_rotationMotor.getEncoder().getPosition());
        double currentSpeed = m_speedMotor.get();
        return new Vector2d(currentSpeed * Math.cos(Math.toRadians(currentAngle)), currentSpeed * Math.sin(Math.toRadians(currentAngle)));
    }

    //change motors to a desired state
    public void setState(Vector2d desiredState){ 
        double targetAngle = desiredState.theta();
        double targetSpeed = desiredState.mag();
        

        m_rotationMotor.getPIDController().setReference(Consts.degreesToRotations(targetAngle), ControlType.kPosition);
        m_speedMotor.set(targetSpeed);
        
        Vector2d currentState = getState();
        targetSpeed = targetSpeed * Math.cos(desiredState.theta() - currentState.theta());
        m_speedMotor.set(targetSpeed);

    }
}
