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
        //get values from suppliers
        double speedX = m_speedX.get();
        double speedY = m_speedY.get();
        double rotation = m_rotation.get();
        
        //apply deadzone on supplier values
        if(Math.abs(speedX) < Consts.JOYSTICK_DEADZONE && Math.abs(speedY) < Consts.JOYSTICK_DEADZONE && Math.abs(rotation) < Consts.JOYSTICK_DEADZONE){
            Swerve.getInstance(Consts.USES_ABS_ENCODER).stop();
            return;
        }

        //create direction vector
        //y is multiplied by -1 because the y axis on the joystick is flipped 
        //rotated by -90 so 0 degrees would be on the front of the joystick
        Vector2d vec = new Vector2d(speedX, speedY * -1).rotate(Math.toRadians(-90));

        //activate the drive function with the s
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
