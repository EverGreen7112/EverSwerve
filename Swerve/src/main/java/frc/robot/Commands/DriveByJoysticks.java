package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Vector2d;

public class DriveByJoysticks extends CommandBase{

    private Supplier<Double> m_speedX, m_speedY, m_rotation;
    private Supplier<Boolean> m_isFieldOriented;

    public DriveByJoysticks(Supplier<Double> speedX, Supplier<Double> speedY, Supplier<Double> rotation, Supplier<Boolean> isFieldOriented){
        m_speedX = speedX;
        m_speedY = speedY;
        m_rotation = rotation;
        m_isFieldOriented = isFieldOriented;
        addRequirements(Swerve.getInstance());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Vector2d vec = new Vector2d(m_speedX.get(), m_speedY.get());
        Swerve.getInstance().drive(vec, m_rotation.get(), m_isFieldOriented.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
