package Subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class Swerve extends SubsystemBase{


    //see math on pdf document for more information
    public void drive(Supplier<Vector2d> directionVec, Supplier<Double> rotationSpeed, Supplier<Boolean> isFieldOriented){
        Vector2d[] rotatedVecs = new Vector2d[4];
        
        for(int i = 0 ;i < 4; i++){
            rotatedVecs[i].set(Consts.physicalMoudulesVector[i]);
            rotatedVecs[i].rotate(Math.toRadians(90));              //adjusting for optimal rotation vector
        }
        double maxMagnitude = rotatedVecs[0].mag();
        
        



    }
    
}
