package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class SwerveModule extends SubsystemBase {

    public CANSparkMax m_speedMotor;
    public CANSparkMax m_rotationMotor;
    public CANCoder m_coder;
    private Vector2d m_desiredState;
    private boolean m_usesAbsEncoder = false;

    public SwerveModule(int movmentPort, int rotationPort) {
        m_speedMotor = new CANSparkMax(movmentPort, MotorType.kBrushless);
        m_rotationMotor = new CANSparkMax(rotationPort, MotorType.kBrushless);

        m_rotationMotor.getPIDController().setP(Consts.WHEEL_ROTATION_KP);
        m_rotationMotor.getPIDController().setI(Consts.WHEEL_ROTATION_KI);
        m_rotationMotor.getPIDController().setD(Consts.WHEEL_ROTATION_KD);

        m_rotationMotor.getEncoder().setPositionConversionFactor(1 / Consts.ROTATION_GEAR_RATIO);

        this.m_desiredState = new Vector2d(0, 0);
    }

    // used only if using CanCoder
    public SwerveModule(int speedPort, int rotationPort, int absoluteEncoderPort, double canCoderOffset) {
        this(speedPort, rotationPort);
        m_coder = new CANCoder(absoluteEncoderPort);
        // m_coder.configMagnetOffset(canCoderOffset, 30);
        m_usesAbsEncoder = true;
        if (m_usesAbsEncoder) {
            m_coder.configFactoryDefault();
            m_rotationMotor.getEncoder().setPosition(m_coder.getAbsolutePosition() / 360);
        }

    }

    public void turnOff() {
        this.m_rotationMotor.set(0);
        this.m_speedMotor.set(0);
    }

    @Override
    public void periodic() {
        // put abs encoder pos in relative encoder of sparkmax
        // div by 360 because position in cancoder is by degrees and sparkmax encoder is
        // by rotations

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
        // desiredState = Consts.optimize(desiredState,
        // Math.toDegrees(currentState.theta()));
        Vector2d currentState = getState();
        double currentAngle = Math.toDegrees(currentState.theta());
        // get target speed and angle from desired state
        double targetAngle = Math.toDegrees(desiredState.theta());
        double targetSpeed = desiredState.mag();

        // rotate module to the target angle
        // if(Math.abs(m_desiredState.theta() - this.getState().theta()) > 5)

        m_rotationMotor.getPIDController().setReference(Consts.degreesToRotations(targetAngle), ControlType.kPosition);

        // drive module by the mag of the desired state(on the current state as the)
        targetSpeed = targetSpeed * Math.cos(desiredState.theta() - currentState.theta());
        m_speedMotor.set(0.1);

    }

}
