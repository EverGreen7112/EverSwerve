package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class SwerveModule extends SubsystemBase {

    public TalonFX m_speedMotor;
    public TalonFX m_rotationMotor;
    public CANCoder m_coder;

    public SwerveModule(int speedPort, int rotationPort) {
        m_speedMotor = new TalonFX(speedPort);
        m_rotationMotor = new TalonFX(rotationPort);

        m_rotationMotor.config_kP(0, Consts.WHEEL_ROTATION_KP);
        m_rotationMotor.config_kI(0, Consts.WHEEL_ROTATION_KI);
        m_rotationMotor.config_kD(0, Consts.WHEEL_ROTATION_KD);
            
        // (1 / (Consts.ROTATION_GEAR_RATIO * 2048));
    }

    // used only if using CanCoder
    public SwerveModule(int speedPort, int rotationPort, int absoluteEncoderPort, double canCoderOffset) {
        this(speedPort, rotationPort);
        m_coder = new CANCoder(absoluteEncoderPort);
        m_coder.configFactoryDefault();
        m_rotationMotor.setSelectedSensorPosition((m_coder.getAbsolutePosition() / 360) * 2048);
    }

    //turn off motors
    public void turnOff() {
        this.m_rotationMotor.set(ControlMode.PercentOutput, 0);
        this.m_speedMotor.set(ControlMode.PercentOutput, 0);
    }

    // get current state of module(magnitude is speed, direction is angle of module)
    public Vector2d getState() {
        double currentAngle = Consts.rotationsToDegrees(m_rotationMotor.getSelectedSensorPosition());
        double currentSpeed = m_speedMotor.getActiveTrajectoryArbFeedFwd();
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
        m_rotationMotor.set(ControlMode.Position,Consts.degreesToRotations(optimizedAngle) * 2048 * Consts.ROTATION_GEAR_RATIO);

        // drive module by the mag of the desired state(on the current state as the)
        targetSpeed = targetSpeed * Math.cos(desiredState.theta() - currentState.theta());
        m_speedMotor.set(ControlMode.PercentOutput ,0.1); //for testing only rotation
    }

}
