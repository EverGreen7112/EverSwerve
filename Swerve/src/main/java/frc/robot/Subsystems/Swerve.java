package frc.robot.Subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenixpro.hardware.CANcoder;

public class Swerve extends SubsystemBase{
    
    private SwerveModule[] m_modules = new SwerveModule[4];
    private CANcoder[] m_encoders = new CANcoder[4];
    private WPI_PigeonIMU m_pigeon;
    private static Swerve m_instance = null;

    public Swerve(){
        m_modules[0] = new SwerveModule(Consts.TOP_LEFT_SPEED_PORT, Consts.TOP_LEFT_ROT_PORT);
        m_modules[1] = new SwerveModule(Consts.TOP_RIGHT_SPEED_PORT, Consts.TOP_RIGHT_ROT_PORT);
        m_modules[2] = new SwerveModule(Consts.DOWN_LEFT_SPEED_PORT, Consts.DOWN_LEFT_ROT_PORT);
        m_modules[3] = new SwerveModule(Consts.DOWN_RIGHT_SPEED_PORT, Consts.DOWN_RIGHT_ROT_PORT);   

        m_encoders[0] = new CANcoder(Consts.TOP_LEFT_CANCODER);
        m_encoders[1] = new CANcoder(Consts.TOP_RIGHT_CANCODER);
        m_encoders[2] = new CANcoder(Consts.DOWN_LEFT_CANCODER);
        m_encoders[3] = new CANcoder(Consts.DOWN_RIGHT_CANCODER);
    }

    public static Swerve getInstance(){
        if(m_instance == null){
            m_instance = new Swerve();
        }
        return m_instance;
    }
    
    
    public void initModules(){
        for(int i = 0; i < m_modules.length; i++){
            m_modules[i].setRotPos(m_encoders[i].getPosition().getValue());
        }
    }

    //see math on pdf document for more information
    public void drive(Supplier<Vector2d> directionVec, Supplier<Double> rotationSpeed, Supplier<Boolean> isFieldOriented){
        
        Vector2d dirVec = directionVec.get();
        if(isFieldOriented.get()){
            dirVec = dirVec.rotate(m_pigeon.getAngle() % 360);
        }
        
        Vector2d[] rotVecs = new Vector2d[4];
        
        for(int i = 0 ;i < 4; i++){
            rotVecs[i] = new Vector2d(Consts.physicalMoudulesVector[i]);
        }

        rotVecs[0].rotate(-45);
        rotVecs[1].rotate(45);
        rotVecs[2].rotate(-45);
        rotVecs[3].rotate(45);

        //find max magnitude
        double maxMagnitude = rotVecs[0].mag();
        for(int i = 1; i < rotVecs.length; i++){
            if(rotVecs[i].mag() > maxMagnitude){
                maxMagnitude = rotVecs[i].mag();
            }
        }

        //normalize by the vector with the biggest magnitude
        for(int i = 0 ;i < rotVecs.length; i++){
            rotVecs[i].mul(1/maxMagnitude); //divide by maxMagnitude
            rotVecs[i].mul(rotationSpeed.get()); //mul by the rotation speed
        }
        
        //add vectors
        Vector2d[] finalVecs = new Vector2d[4]; 
        for(int i = 0; i < finalVecs.length; i++){
            finalVecs[i] = new Vector2d(rotVecs[i]);
            finalVecs[i].add(dirVec);
        }

        //find max magnitude
        maxMagnitude = finalVecs[0].mag();
        for(int i = 1; i < finalVecs.length; i++){
            if(finalVecs[i].mag() > maxMagnitude){
                maxMagnitude = finalVecs[i].mag();
            }
        }

        //normalize by the vector with the biggest magnitude
        for(int i = 0 ;i < finalVecs.length; i++){
            finalVecs[i].mul(1/maxMagnitude); //divide by maxMagnitude
        }

        //set target state of module
        for(int i = 0; i < finalVecs.length; i++){
            m_modules[i].setState(finalVecs[i]);
        }        
    }    
}