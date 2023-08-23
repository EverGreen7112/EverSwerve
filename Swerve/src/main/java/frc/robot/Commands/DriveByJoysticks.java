package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class DriveByJoysticks extends CommandBase{

    private Supplier<Double> m_speedX, m_speedY, m_rotation;
    private Supplier<Boolean> m_isFieldOriented;
    private boolean m_usesAbsEncoder;

    public DriveByJoysticks(Supplier<Double> speedX, Supplier<Double> speedY, Supplier<Double> rotation, Supplier<Boolean> isFieldOriented, boolean usesAbsEncoder){
        m_speedX = speedX;
        m_speedY = speedY;
        m_rotation = rotation;
        m_isFieldOriented = isFieldOriented;
        m_usesAbsEncoder = usesAbsEncoder;
    }

    @Override
    public void initialize() {
        addRequirements(Swerve.getInstance(m_usesAbsEncoder));
    }

    @Override
    public void execute() {
        double speedX = m_speedX.get();
        double speedY = m_speedY.get();
        double rotation = m_rotation.get();

        if(Math.abs(speedX) < Consts.JOYSTICK_DEADZONE && Math.abs(speedY) < Consts.JOYSTICK_DEADZONE && Math.abs(rotation) < Consts.JOYSTICK_DEADZONE){
            Swerve.getInstance(true).stop();
            return;
        }
        Vector2d vec = new Vector2d(speedX * Consts.MAX_SPEED, speedY * Consts.MAX_SPEED * -1).rotate(Math.toRadians(-90));
        Swerve.getInstance(m_usesAbsEncoder).drive(vec, rotation, m_isFieldOriented.get());
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
