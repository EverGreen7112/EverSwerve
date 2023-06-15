package Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class SwerveModule extends SubsystemBase{

    private CANSparkMax m_speedMotor;
    private CANSparkMax m_rotationMotor;
    private Vector2d m_desiredState;

    public SwerveModule(int speedPort, int rotationPort){
        m_speedMotor = new CANSparkMax(speedPort, MotorType.kBrushless);
        m_rotationMotor = new CANSparkMax(rotationPort, MotorType.kBrushless);
        m_rotationMotor.getPIDController().setP(0);
        m_rotationMotor.getPIDController().setI(0);
        m_rotationMotor.getPIDController().setD(0);
        m_speedMotor.getPIDController().setP(0);
        m_speedMotor.getPIDController().setI(0);
        m_speedMotor.getPIDController().setD(0);
    }

    public Vector2d getState(){
        double currentAngle = Consts.ticksToAngle(m_rotationMotor.getEncoder().getPosition(), Consts.ticksPerRev);
        double currentSpeed = Consts.rpmToMs(Consts.wheelRadius, m_speedMotor.getEncoder().getVelocity());
        return new Vector2d(currentSpeed * Math.sin(Math.toRadians(currentAngle)), currentSpeed * Math.cos(Math.toRadians(currentAngle)));
    }

    //set PID controllers for CanSpark motors
    public void setState(Vector2d desiredState){
        m_desiredState = desiredState;
        double targetAngle = m_desiredState.theta();
        double targetSpeed = m_desiredState.mag();
        m_rotationMotor.getPIDController().setReference(Consts.angleToTicks(Math.toDegrees(targetAngle), Consts.ticksPerRev), ControlType.kPosition);
        m_speedMotor.getPIDController().setReference(targetSpeed, ControlType.kDutyCycle);
    }
}
