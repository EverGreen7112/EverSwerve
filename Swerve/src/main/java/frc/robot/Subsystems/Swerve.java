package frc.robot.Subsystems;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Swerve extends SubsystemBase{
    
    private SwerveModule[] m_modules = new SwerveModule[4];
    private PigeonIMU m_pigeon; 
    private static Swerve m_instance = null;

    public Swerve(){
        m_modules[0] = new SwerveModule(Consts.TOP_LEFT_SPEED_PORT, Consts.TOP_LEFT_ROT_PORT, Consts.TOP_LEFT_CANCODER);
        m_modules[1] = new SwerveModule(Consts.TOP_RIGHT_SPEED_PORT, Consts.TOP_RIGHT_ROT_PORT, Consts.TOP_RIGHT_CANCODER);
        m_modules[2] = new SwerveModule(Consts.DOWN_LEFT_SPEED_PORT, Consts.DOWN_LEFT_ROT_PORT, Consts.DOWN_LEFT_CANCODER);
        m_modules[3] = new SwerveModule(Consts.DOWN_RIGHT_SPEED_PORT, Consts.DOWN_RIGHT_ROT_PORT, Consts.DOWN_RIGHT_CANCODER);        

        m_pigeon = new PigeonIMU(Consts.PIGEON);
    }

    public static Swerve getInstance(){
        if(m_instance == null){
            m_instance = new Swerve();
        }
        return m_instance;
    }
    
    @Override
    public void periodic() {
        Vector2d topLeftState = m_modules[0].getState();
        SmartDashboard.putString("top left module", "speed: " + topLeftState.mag() + " angle: " + topLeftState.theta());

        Vector2d topRightState = m_modules[1].getState();
        SmartDashboard.putString("top left module", "speed: " + topRightState.mag() + " angle: " + topRightState.theta());

        Vector2d downLeftState = m_modules[2].getState();
        SmartDashboard.putString("top left module", "speed: " + downLeftState.mag() + " angle: " + downLeftState.theta());

        Vector2d downRightState = m_modules[3].getState();
        SmartDashboard.putString("top left module", "speed: " + downRightState.mag() + " angle: " + downRightState.theta());
        
        //put absolute encoder value in relative encoder of SparkMax 
        for(int i = 0; i < 4; i++){
            m_modules[i].setAbsPosInSpark();;
        }
    }

    //see math on pdf document for more information
    public void drive(Supplier<Vector2d> directionVec, Supplier<Double> spinSpeed, Supplier<Boolean> isFieldOriented){
        Vector2d dirVec = directionVec.get();
        //i thought we might need to rotate the vector to the opposite direction, can you think about it? 
        if(isFieldOriented.get()){
            dirVec = dirVec.rotate(-(m_pigeon.getYaw() % 360));
        }
        
        Vector2d[] rotVecs = new Vector2d[4];
        
        for(int i = 0 ;i < 4; i++){
            rotVecs[i] = new Vector2d(Consts.physicalMoudulesVector[i]); 
            rotVecs[i].rotate(Math.toDegrees(90));
        }

        //find magnitude
        double mag = rotVecs[0].mag(); 

        //normalize by the vector with the biggest magnitude
        for(int i = 0 ;i < rotVecs.length; i++){
            rotVecs[i].mul(1/mag); //normalize
            rotVecs[i].mul(spinSpeed.get()); //mul by the rotation speed
        }
        
        //add vectors
        Vector2d[] finalVecs = new Vector2d[4]; 
        for(int i = 0; i < finalVecs.length; i++){
            finalVecs[i] = new Vector2d(rotVecs[i]);
            finalVecs[i].add(dirVec);
        }

        //find max magnitude
        mag = finalVecs[0].mag();
        for(int i = 1; i < finalVecs.length; i++){
            mag = Math.max(mag, finalVecs[i].mag());
        }

        //normalize by the vector with the biggest magnitude
        for(int i = 0 ;i < finalVecs.length; i++){
            finalVecs[i].mul(1/mag); //divide by maxMagnitude
        }

        //set target state of module
        for(int i = 0; i < finalVecs.length; i++){
            m_modules[i].setState(finalVecs[i]);
        }        
    }    
}
