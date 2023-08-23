package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class SwerveModule extends SubsystemBase {

    public TalonFX m_speedMotor;
    public TalonFX m_rotationMotor;
    public CANCoder m_coder;
    private Vector2d desiredState = new Vector2d(0, 0);

    public SwerveModule(int speedPort, int rotationPort) {
        m_speedMotor = new TalonFX(speedPort);
        m_rotationMotor = new TalonFX(rotationPort);
        m_rotationMotor.configFactoryDefault();
        
        m_rotationMotor.setNeutralMode(NeutralMode.Coast);
        m_rotationMotor.config_kP(0, Consts.WHEEL_ROTATION_KP);
        m_rotationMotor.config_kI(0, Consts.WHEEL_ROTATION_KI);
        m_rotationMotor.config_kD(0, Consts.WHEEL_ROTATION_KD);

    }

    // used only if using CanCoder
    public SwerveModule(int speedPort, int rotationPort, int absoluteEncoderPort, double canCoderOffset) {
        this(speedPort, rotationPort);
        m_coder = new CANCoder(absoluteEncoderPort);
        m_coder.configFactoryDefault();
        m_coder.configMagnetOffset(canCoderOffset);
        m_rotationMotor.setSelectedSensorPosition(m_coder.getAbsolutePosition() / (360.0 / (Consts.ROTATION_GEAR_RATIO * 2048.0)));
    }

    @Override
    public void periodic() {       }

    //turn off motors
    public void turnOff() {
        this.m_rotationMotor.set(ControlMode.PercentOutput, 0);
        this.m_speedMotor.set(ControlMode.PercentOutput, 0);
    }

    // get current state of module(magnitude is speed, direction is angle of module)
    public Vector2d getState() {
        double currentAngle = Consts.rotationsToDegrees(m_rotationMotor.getSelectedSensorPosition() / 2048 / Consts.ROTATION_GEAR_RATIO);
        double currentSpeed = m_speedMotor.getMotorOutputPercent();
        return new Vector2d(currentSpeed * Math.cos(Math.toRadians(currentAngle)),
                currentSpeed * Math.sin(Math.toRadians(currentAngle)));
    }

    // change motors to the desired state
    public void setState(Vector2d desiredState) {
        Vector2d currentState = getState();
        
        double targetAngle = Math.toDegrees(desiredState.theta());
        double targetSpeed = desiredState.mag();

        SmartDashboard.putNumber("target angle", targetAngle);
        m_rotationMotor.set(ControlMode.Position, targetAngle / (360.0 / (Consts.ROTATION_GEAR_RATIO * 2048.0)));
        m_speedMotor.set(ControlMode.PercentOutput, targetSpeed * Consts.MAX_SPEED);
        SmartDashboard.putNumber("speed", targetSpeed);
    }

}
