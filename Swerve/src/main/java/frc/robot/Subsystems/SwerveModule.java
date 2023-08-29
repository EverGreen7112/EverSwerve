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

    public CANSparkMax m_speedMotor;
    public CANSparkMax m_rotationMotor;
    public CANCoder m_coder;

    public SwerveModule(int speedPort, int rotationPort) {
        m_speedMotor = new CANSparkMax(speedPort, MotorType.kBrushless);
        m_rotationMotor = new CANSparkMax(rotationPort, MotorType.kBrushless);
        m_rotationMotor.restoreFactoryDefaults();
        
        m_rotationMotor.setIdleMode(IdleMode.kCoast);
        m_rotationMotor.getPIDController().setP(Consts.WHEEL_ROTATION_KP);
        m_rotationMotor.getPIDController().setI(Consts.WHEEL_ROTATION_KI);
        m_rotationMotor.getPIDController().setD(Consts.WHEEL_ROTATION_KD);

    }

    // used only if using CanCoder
    public SwerveModule(int speedPort, int rotationPort, int absoluteEncoderPort, double canCoderOffset) {
        this(speedPort, rotationPort);
        m_coder = new CANCoder(absoluteEncoderPort);
        m_coder.configFactoryDefault();
        m_coder.configMagnetOffset(canCoderOffset);
        m_rotationMotor.getEncoder().setPosition(m_coder.getAbsolutePosition() / (360.0 / (Consts.ROTATION_GEAR_RATIO)));
    }

    @Override
    public void periodic() {}

    //turn off motors
    public void turnOff() {
        this.m_rotationMotor.set(0);
        this.m_speedMotor.set(0);
    }

    // get current state of module(magnitude is speed, direction is angle of module)
    public Vector2d getState() {
        double currentAngle = Consts.rotationsToDegrees(m_rotationMotor.getEncoder().getPosition() * 360.0 / Consts.ROTATION_GEAR_RATIO);
        double currentSpeed = m_speedMotor.getAppliedOutput();
        return new Vector2d(currentSpeed * Math.cos(Math.toRadians(currentAngle)),
                currentSpeed * Math.sin(Math.toRadians(currentAngle)));
    }

    // change motors to the desired state
    public void setState(Vector2d desiredState) {
        Vector2d currentState = getState();
        
        double targetAngle = Math.toDegrees(desiredState.theta());
        double targetSpeed = desiredState.mag();

        m_rotationMotor.getPIDController().setReference(targetAngle / (360.0 / (Consts.ROTATION_GEAR_RATIO)), ControlType.kPosition);
        m_speedMotor.set(targetSpeed);
    }

}
