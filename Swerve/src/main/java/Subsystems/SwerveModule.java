package Subsystems;

import com.revrobotics.CANSparkMax;
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
    }

    public Vector2d getState(){
        double currentAngle = Consts.ticksToAngle(m_rotationMotor.getEncoder().getPosition(), Consts.ticksPerRev);
        double currentSpeed = Consts.rpmToMs(Consts.wheelRadius, m_speedMotor.getEncoder().getVelocity());
        return new Vector2d(currentSpeed * Math.sin(Math.toRadians(currentAngle)), currentSpeed * Math.cos(Math.toRadians(currentAngle)));
    }

    public void setState(Vector2d desiredState){
        m_desiredState = desiredState;
    }


}
