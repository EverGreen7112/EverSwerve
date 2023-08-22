package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class SwerveModule extends SubsystemBase {

    public CANSparkMax m_speedMotor;
    public CANSparkMax m_rotationMotor;
    public CANCoder m_coder;

    public SwerveModule(int movmentPort, int rotationPort) {
        m_speedMotor = new CANSparkMax(movmentPort, MotorType.kBrushless);
        m_rotationMotor = new CANSparkMax(rotationPort, MotorType.kBrushless);

        m_rotationMotor.getPIDController().setP(Consts.WHEEL_ROTATION_KP);
        m_rotationMotor.getPIDController().setI(Consts.WHEEL_ROTATION_KI);
        m_rotationMotor.getPIDController().setD(Consts.WHEEL_ROTATION_KD);

        m_rotationMotor.getEncoder().setPositionConversionFactor(1 / Consts.ROTATION_GEAR_RATIO);
    }

    // used only if using CanCoder
    public SwerveModule(int speedPort, int rotationPort, int absoluteEncoderPort, double canCoderOffset) {
        this(speedPort, rotationPort);
        m_coder = new CANCoder(absoluteEncoderPort);
        m_coder.configFactoryDefault();
        m_rotationMotor.getEncoder().setPosition(m_coder.getAbsolutePosition() / 360);
    }

    //turn off motors
    public void turnOff() {
        this.m_rotationMotor.set(0);
        this.m_speedMotor.set(0);
    }

    // get current state of module(magnitude is speed, direction is angle of module)
    public Vector2d getState() {
        double currentAngle = Consts.rotationsToDegrees(m_rotationMotor.getEncoder().getPosition());
        double currentSpeed = m_speedMotor.get();
        return new Vector2d(currentSpeed * Math.cos(Math.toRadians(currentAngle)),
                currentSpeed * Math.sin(Math.toRadians(currentAngle)));
    }

    // change motors to the desired state
    public void setState(Vector2d desiredState) {
        // get current module vector
        Vector2d currentState = getState();
        
        // get target speed and angle from desired state
        double targetAngle = Math.toDegrees(desiredState.theta());
        double targetSpeed = desiredState.mag();

        double optimizedAngle = Math.toDegrees(currentState.theta()) + Consts.closestAngle(Math.toDegrees(currentState.theta()), targetAngle);
        // rotate module to the target angle
        m_rotationMotor.getPIDController().setReference(Consts.degreesToRotations(optimizedAngle), ControlType.kPosition);

        // drive module by the mag of the desired state(on the current state as the)
        targetSpeed = targetSpeed * Math.cos(desiredState.theta() - currentState.theta());
        m_speedMotor.set(0.1); //for testing only rotation
    }

}
