package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Funcs;
import frc.robot.Utils.Vector2d;

public class DriveByJoysticks extends CommandBase implements Constants{

    private Supplier<Double> m_speedX, m_speedY, m_rotation;
    private Supplier<Boolean> m_isFieldOriented;
    private boolean m_usesAbsEncoder;
    private double m_currentTime;

    public DriveByJoysticks(Supplier<Double> speedX, Supplier<Double> speedY, Supplier<Double> rotation, Supplier<Boolean> isFieldOriented, boolean usesAbsEncoder){
        addRequirements(Swerve.getInstance(SwerveValues.USES_ABS_ENCODER));
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
        if(Math.abs(speedX) < JoystickValues.JOYSTICK_DEADZONE)
            speedX = 0;
        if(Math.abs(speedY) < JoystickValues.JOYSTICK_DEADZONE)
            speedY = 0; 
        if(Math.abs(rotation) < JoystickValues.JOYSTICK_DEADZONE)
            rotation = 0;

        //round values
        rotation = Funcs.roundAfterDecimalPoint(rotation, 2);
        speedX = Funcs.roundAfterDecimalPoint(speedX, 2);
        speedY = Funcs.roundAfterDecimalPoint(speedY, 2);

        //rotate robot according to rotation supplier   
        Swerve.getInstance(SwerveValues.USES_ABS_ENCODER).rotateBy(SpeedValues.MAX_ANGULAR_SPEED.get() * rotation * deltaTime);
        //create drive vector
        Vector2d vec = new Vector2d(-speedX, speedY);
        //make sure mag never goes over 1 so driving in all directions will be the same speed
        if(vec.mag() > 1){
            vec.normalise();
        }
        //drive
        Swerve.getInstance(m_usesAbsEncoder).drive(vec, m_isFieldOriented.get());
        //update current time
        m_currentTime = System.currentTimeMillis() / 1000.0;
    }

}
