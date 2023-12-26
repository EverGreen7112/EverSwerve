package frc.robot.Commands;

import java.util.concurrent.CyclicBarrier;
import java.util.function.Supplier;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.SystemMenuBar;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class DriveByJoysticks extends CommandBase{

    private Supplier<Double> m_speedX, m_speedY, m_rotation;
    private Supplier<Boolean> m_isFieldOriented;
    private boolean m_usesAbsEncoder;
    private double m_currentTime;

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
        m_currentTime = System.currentTimeMillis() / 1000.0;
    }

    @Override
    public void execute() {
        //get values from suppliers
        double speedX = m_speedX.get();
        double speedY = m_speedY.get();
        double rotation = m_rotation.get();
        
        double deltaTime = (System.currentTimeMillis() / 1000.0) - m_currentTime;

        //apply deadzone on supplier values
        if(Math.abs(speedX) < Consts.JOYSTICK_DEADZONE)
            speedX = 0;
        if(Math.abs(speedY) < Consts.JOYSTICK_DEADZONE)
            speedY = 0; 
        if(Math.abs(rotation) < Consts.JOYSTICK_DEADZONE)
            rotation = 0;

        //round values
        rotation = Consts.roundAfterDecimalPoint(rotation, 2);
        speedX = Consts.roundAfterDecimalPoint(speedX, 2);
        speedY = Consts.roundAfterDecimalPoint(speedY, 2);

        //rotate robot according to rotation supplier
        Swerve.getInstance(Consts.USES_ABS_ENCODER).rotateBy(360 * (Consts.MAX_ANGULAR_SPEED.get() / Consts.ROBOT_BOUNDING_CIRCLE_PERIMETER) * rotation * deltaTime);
        //create drive vector
        Vector2d vec = new Vector2d(-speedX, speedY);
        //drive
        Swerve.getInstance(m_usesAbsEncoder).drive(vec, m_isFieldOriented.get());
        //update current time
        m_currentTime = System.currentTimeMillis() / 1000.0;
    }

}
