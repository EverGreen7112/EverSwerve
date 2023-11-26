package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class DriveByJoysticks extends CommandBase{

    private Supplier<Double> m_speedX, m_speedY, m_rotation;
    private Supplier<Boolean> m_isFieldOriented;
    private PIDController m_spinPIDcontroller;
    private Swerve m_swerveInstance;
    private double m_targetAngle;

    public DriveByJoysticks(Supplier<Double> speedX, Supplier<Double> speedY, Supplier<Double> rotation, Supplier<Boolean> isFieldOriented, boolean usesAbsEncoder){
        m_speedX = speedX;
        m_speedY = speedY;
        m_rotation = rotation;
        m_isFieldOriented = isFieldOriented;
        m_spinPIDcontroller = new PIDController(Consts.SPIN_ANGLE_KP.get(), Consts.SPIN_ANGLE_KI, Consts.SPIN_ANGLE_KD.get());
        
        m_swerveInstance = Swerve.getInstance(usesAbsEncoder);
    }

    @Override
    public void initialize() {
        addRequirements(m_swerveInstance);
        m_targetAngle = m_swerveInstance.getGyro().getYaw();
    }

    @Override
    public void execute() {


        //current robot angle
        double currentAngle = m_swerveInstance.getGyro().getYaw();
        //get values from suppliers
        double speedX = m_speedX.get();
        double speedY = m_speedY.get();
        double rotation = m_rotation.get();
       
        if(Math.abs(speedX) < Consts.JOYSTICK_DEADZONE && Math.abs(speedY) < Consts.JOYSTICK_DEADZONE && Math.abs(rotation) < Consts.JOYSTICK_DEADZONE){
            m_swerveInstance.stop();
            m_targetAngle = currentAngle;
            return;
        }
        m_spinPIDcontroller.setPID(Consts.SPIN_ANGLE_KP.get(), Consts.SPIN_ANGLE_KI, Consts.SPIN_ANGLE_KD.get());

        //update robot's target angle
        m_targetAngle += Consts.SPIN_SPEED * rotation;

        double optimizedAngle = currentAngle + Consts.closestAngle(currentAngle, m_targetAngle);
        double output = -Consts.clamp(m_spinPIDcontroller.calculate(currentAngle, optimizedAngle), -1, 1);
        
        SmartDashboard.putNumber("target", m_targetAngle);
        SmartDashboard.putNumber("output", output);
        SmartDashboard.putNumber("current angle", currentAngle);
        SmartDashboard.putNumber("optimized", optimizedAngle);
        //create direction vector
        //y is multiplied by -1 because the y axis on the joystick is flipped 
        //rotated by -90 so 0 degrees would be on the front of the joystick
        Vector2d vec = new Vector2d(speedX, speedY * -1).rotate(Math.toRadians(-90));


        //activate the drive function
        m_swerveInstance.drive(vec, output, m_isFieldOriented.get());
                
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
