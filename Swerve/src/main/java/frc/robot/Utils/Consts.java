package frc.robot.Utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Swerve;

public class Consts {

  public static final boolean USES_ABS_ENCODER = false;

    //speed values in m/s
    public static final Supplier<Double> MAX_SPEED = new Supplier<Double>() {
      @Override
      public Double get() {
        return SmartDashboard.getNumber("max speed", 1);
      }
    };

    //speed values in m/s
    public static final Supplier<Double> MAX_ANGULAR_SPEED = new Supplier<Double>() {
      @Override
      public Double get() {
        return SmartDashboard.getNumber("max angular speed", 5);
      }
    };    

    //chassis motors
    public static final int TOP_LEFT_DRIVE_PORT = 6;
    public static final int TOP_RIGHT_DRIVE_PORT = 8;
    public static final int DOWN_LEFT_DRIVE_PORT = 4;
    public static final int DOWN_RIGHT_DRIVE_PORT = 10;
    public static final int TOP_LEFT_STEERING_PORT = 7;
    public static final int TOP_RIGHT_STEERING_PORT = 9;
    public static final int DOWN_LEFT_STEERING_PORT = 5;
    public static final int DOWN_RIGHT_STEERING_PORT = 11;

    //chassis encoders
    public static final int TOP_LEFT_CANCODER = 0;
    public static final int TOP_RIGHT_CANCODER = 2;
    public static final int DOWN_LEFT_CANCODER = 1;
    public static final int DOWN_RIGHT_CANCODER = 3;

    //cancoder offset(remember to update with robot!!)
    //remember to sub your offset from 360
    public static final double TOP_RIGHT_CANCODER_OFFSET = 360 - 126.9140625; //360 - 4.218
    public static final double TOP_LEFT_CANCODER_OFFSET =  360 - 116.54296875;//360 - 247.24
    public static final double DOWN_RIGHT_CANCODER_OFFSET = 360 - 18.80859375; //360 - 72.246
    public static final double DOWN_LEFT_CANCODER_OFFSET = 360 - 250.6640625;// 360 - 238.71
    
    //joysticks
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final double JOYSTICK_DEADZONE = 0.2;

    //pid values
    public static final double WHEEL_ANGLE_KP = 0.01; //0.08
    public static final double WHEEL_ANGLE_KI = 0.0;//0.000007
    public static final double WHEEL_ANGLE_KD = 0.0;

    public static final double WHEEL_VELOCITY_KP = 0.1;
    public static final double WHEEL_VELOCITY_KI = 0.001;
    public static final double WHEEL_VELOCITY_KD = 0.005;


    //chassis size
    public static final double FRONT_WHEEL_DIST_METERS = 0.57;
    public static final double SIDE_WHEEL_DIST_METERS = 0.57;
    public static final double WHEEL_PERIMETER = Math.PI * 0.095;
    //swerve vectors
    public static final Vector2d TOP_RIGHT = new Vector2d((FRONT_WHEEL_DIST_METERS / 2), (SIDE_WHEEL_DIST_METERS / 2)); 
    public static final Vector2d TOP_LEFT = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2), SIDE_WHEEL_DIST_METERS / 2);
    public static final Vector2d DOWN_RIGHT = new Vector2d(FRONT_WHEEL_DIST_METERS / 2, -(SIDE_WHEEL_DIST_METERS / 2));
    public static final Vector2d DOWN_LEFT = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2), -(SIDE_WHEEL_DIST_METERS / 2));

    //array of physical module vectors
    public static final Vector2d[] physicalMoudulesVector = { TOP_RIGHT, TOP_LEFT, DOWN_RIGHT, DOWN_LEFT};//array of vectors from robot center to swerves module

    //module gear ratios
    public static final double DRIVE_GEAR_RATIO =  1 / 8.14; //L1
    public static final double STEERING_GEAR_RATIO = 1 / 12.8;

     //speed values
    public static final Supplier<Double> HEADING_KP = new Supplier<Double>() {
      @Override
      public Double get() {
        return SmartDashboard.getNumber("heading kp", 0);
      }
    };  
    public static final double HEADING_KI = 0;
    public static final Supplier<Double> HEADING_KD = new Supplier<Double>() {
      @Override
      public Double get() {
        return SmartDashboard.getNumber("heading kd", 0);
      }
    };  

    public static double rpmToMs(double wheelRadius, double rpm){
      double rps = rpm / 60;
      return wheelRadius * 2 * Math.PI * rps;
    }

    public static double closestAngle(double a, double b) {
      // get direction
      double dir = modulo(b, 360.0) - modulo(a, 360.0);

      // convert from -360 to 360 to -180 to 180
      if (Math.abs(dir) > 180.0) {
          dir = -(Math.signum(dir) * 360.0) + dir;
      }
      return dir;
    }

    public static double modulo(double a, double b) {
      return ((a % b) + b) % b;
    }

   

}
