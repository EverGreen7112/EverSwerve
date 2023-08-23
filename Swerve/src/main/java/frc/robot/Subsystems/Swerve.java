package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

public class Swerve extends SubsystemBase {

    public SwerveModule[] m_modules = new SwerveModule[Consts.physicalMoudulesVector.length];
    private AHRS m_navx;
    private static Swerve m_instance = null;

    public Swerve(boolean usesAbsEncoder) {

        if (usesAbsEncoder) {
            m_modules[0] = new SwerveModule(Consts.TOP_RIGHT_SPEED_PORT, Consts.TOP_RIGHT_ROT_PORT,
                    Consts.TOP_RIGHT_CANCODER, Consts.TOP_RIGHT_CANCODER_OFFSET);
            m_modules[1] = new SwerveModule(Consts.TOP_LEFT_SPEED_PORT, Consts.TOP_LEFT_ROT_PORT,
                    Consts.TOP_LEFT_CANCODER, Consts.TOP_LEFT_CANCODER_OFFSET);
            m_modules[2] = new SwerveModule(Consts.DOWN_RIGHT_SPEED_PORT, Consts.DOWN_RIGHT_ROT_PORT,
                    Consts.DOWN_RIGHT_CANCODER, Consts.DOWN_RIGHT_CANCODER_OFFSET);
            m_modules[3] = new SwerveModule(Consts.DOWN_LEFT_SPEED_PORT, Consts.DOWN_LEFT_ROT_PORT,
                    Consts.DOWN_LEFT_CANCODER, Consts.DOWN_LEFT_CANCODER_OFFSET);
        } else {
            m_modules[0] = new SwerveModule(Consts.TOP_RIGHT_SPEED_PORT, Consts.TOP_RIGHT_ROT_PORT);
            m_modules[1] = new SwerveModule(Consts.TOP_LEFT_SPEED_PORT, Consts.TOP_LEFT_ROT_PORT);
            m_modules[2] = new SwerveModule(Consts.DOWN_RIGHT_SPEED_PORT, Consts.DOWN_RIGHT_ROT_PORT);
            m_modules[3] = new SwerveModule(Consts.DOWN_LEFT_SPEED_PORT, Consts.DOWN_LEFT_ROT_PORT);
        }
        m_navx = new AHRS(SerialPort.Port.kMXP);
    }

    public static Swerve getInstance(boolean usesAbsEncoder) {
        if (m_instance == null) {
            m_instance = new Swerve(usesAbsEncoder);
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        Vector2d topLeftState = m_modules[1].getState();

        SmartDashboard.putNumber("yaw", m_navx.getYaw());
        SmartDashboard.putString("top left module",
                 "speed: " + topLeftState.mag() + " angle: " + Math.toDegrees(topLeftState.theta()));

        Vector2d topRightState = m_modules[0].getState();
        SmartDashboard.putString("top right module",
                "speed: " + topRightState.mag() + " angle: " + Math.toDegrees(topRightState.theta()));

        Vector2d downLeftState = m_modules[3].getState();
        SmartDashboard.putString("down left module",
                "speed: " + downLeftState.mag() + " angle: " + Math.toDegrees(downLeftState.theta()));

        Vector2d downRightState = m_modules[2].getState();
        SmartDashboard.putString("down right module",
                "speed: " + downRightState.mag() + " angle: " + Math.toDegrees(downRightState.theta()));

        for (int i = 0; i < m_modules.length; i++) {
            SmartDashboard.putNumber("sparkmax encoder " + i, (m_modules[i].m_rotationMotor.getSelectedSensorPosition() / 2048) * 360);
            SmartDashboard.putNumber("Can coder " + i, m_modules[i].m_coder.getAbsolutePosition());
        }
    }

    // see math on pdf document for more information
    public void drive(Vector2d directionVec, double spinSpeed, boolean isFieldOriented) {
        Vector2d dirVec = directionVec;
        if (isFieldOriented) {
            dirVec = dirVec.rotate((-m_navx.getYaw()));
        }

        Vector2d[] rotVecs = new Vector2d[m_modules.length];

        for (int i = 0; i < rotVecs.length; i++) {
            rotVecs[i] = new Vector2d(Consts.physicalMoudulesVector[i]);
            rotVecs[i].rotate(Math.toRadians(0));
        }

        // we need the max magnitude and because all of the magnitudes are equals there
        // is no reason to search for the biggest one
        double mag = rotVecs[0].mag();

        // normalize by the vector with the biggest magnitude
        for (int i = 0; i < rotVecs.length; i++) {
            rotVecs[i].mul(1 / mag); // normalize
            rotVecs[i].mul(spinSpeed); // mul by the rotation speed
        }

        // add vectors
        Vector2d[] finalVecs = new Vector2d[m_modules.length];
        for (int i = 0; i < finalVecs.length; i++) {
            finalVecs[i] = new Vector2d(rotVecs[i]);
            finalVecs[i].add(dirVec);
        }

        // find max magnitude
        mag = finalVecs[0].mag();
        for (int i = 1; i < finalVecs.length; i++) {
            mag = Math.max(mag, finalVecs[i].mag());
        }

        // normalize by the vector with the biggest magnitude
        for (int i = 0; i < finalVecs.length; i++) {
            finalVecs[i].mul(1 / mag); // divide by maxMagnitude
        }

        // set target state of module
        for (int i = 0; i < finalVecs.length; i++) {
            m_modules[i].setState(finalVecs[i]);
        }
    }

    // untested alternative way to calculate module states
    public void driveByMats(Vector2d directionVec, double spinSpeed, boolean isFieldOriented) {
        Vector2d dirVec = directionVec;
        if (isFieldOriented) {
            dirVec = dirVec.rotate(-(m_navx.getYaw() % 360));
        }

        for (int i = 0; i < m_modules.length; i++) {
            Vector2d driveVec = new Vector2d(dirVec.x - Consts.physicalMoudulesVector[i].y * spinSpeed,
                    spinSpeed * Consts.physicalMoudulesVector[i].x + dirVec.y);
            m_modules[i].setState(driveVec);
        }

    }

    public void stop() {
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].turnOff();
        }
    }
}
