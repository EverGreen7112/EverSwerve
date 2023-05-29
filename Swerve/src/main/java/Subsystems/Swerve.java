package Subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class Swerve extends SubsystemBase{

    private AHRS m_navx = new AHRS(Port.kMXP);
    //see math on pdf document for more information
    public void drive(Supplier<Vector2d> directionVec, Supplier<Double> rotationSpeed, Supplier<Boolean> isFieldOriented){
        
        Vector2d dirVec = directionVec.get();
        if(isFieldOriented.get()){
            dirVec = dirVec.rotate(m_navx.getAngle());
        }
        
        Vector2d[] rotVecs = new Vector2d[4];
        
        for(int i = 0 ;i < 4; i++){
            rotVecs[i].set(Consts.physicalMoudulesVector[i]);
            rotVecs[i].rotate(Math.toRadians(45));              //adjusting for optimal rotation vector
        }

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

    }    
}
