package frc.robot.Utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Consts {

    //speed values
    public static final Supplier<Double> MAX_SPEED = new Supplier<Double>() {
      @Override
      public Double get() {
        return SmartDashboard.getNumber("max speed", 0.2);
      }
    };

    //chassis motors
    public static final int TOP_LEFT_SPEED_PORT = 6;
    public static final int TOP_RIGHT_SPEED_PORT = 8;
    public static final int DOWN_LEFT_SPEED_PORT = 4;
    public static final int DOWN_RIGHT_SPEED_PORT = 10;
    public static final int TOP_LEFT_ROT_PORT = 7;
    public static final int TOP_RIGHT_ROT_PORT = 9;
    public static final int DOWN_LEFT_ROT_PORT = 5;
    public static final int DOWN_RIGHT_ROT_PORT = 11;

    //chassis encoders
    public static final int TOP_LEFT_CANCODER = 0;
    public static final int TOP_RIGHT_CANCODER = 2;
    public static final int DOWN_LEFT_CANCODER = 1;
    public static final int DOWN_RIGHT_CANCODER = 3;

    //gyro id
    public static final int PIGEON = 9;

    //cancoder offset(remember to update with robot!!)
    //remember to sub your offset from 360
    public static final double TOP_RIGHT_CANCODER_OFFSET = 360 - 227.98828125; 
    public static final double TOP_LEFT_CANCODER_OFFSET =  360 - 334.6875;
    public static final double DOWN_RIGHT_CANCODER_OFFSET = 360 - 129.19921875;
    public static final double DOWN_LEFT_CANCODER_OFFSET = 360 - 268.857421875;
    //joysticks
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final double JOYSTICK_DEADZONE = 0.2;

    //pid values
    public static final double WHEEL_ROTATION_KP = 0.01; //0.08
    public static final double WHEEL_ROTATION_KI = 0.0;//0.000007
    public static final double WHEEL_ROTATION_KD = 0.0;

    //chassis size
    public static final double FRONT_WHEEL_DIST_METERS = 0.57;
    public static final double SIDE_WHEEL_DIST_METERS = 0.57;

    //swerve vectors
    public static final Vector2d TOP_RIGHT = new Vector2d((FRONT_WHEEL_DIST_METERS / 2), (SIDE_WHEEL_DIST_METERS / 2)); 
    public static final Vector2d TOP_LEFT = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2), SIDE_WHEEL_DIST_METERS / 2);
    public static final Vector2d DOWN_RIGHT = new Vector2d(FRONT_WHEEL_DIST_METERS / 2, -(SIDE_WHEEL_DIST_METERS / 2));
    public static final Vector2d DOWN_LEFT = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2), -(SIDE_WHEEL_DIST_METERS / 2));

    //array of physical module vectors
    public static final Vector2d[] physicalMoudulesVector = { TOP_RIGHT, TOP_LEFT, DOWN_RIGHT, DOWN_LEFT};//array of vectors from robot center to swerves module

    //gear ratios
    public static final double DRIVE_GEAR_RATIO =  1 / 8.14; //L1
    public static final double ROTATION_GEAR_RATIO = 1 / 12.8;

    //robot yaw angle PID
    public static final double SPIN_SPEED = 1.5;
    
 //speed values
 public static final Supplier<Double> SPIN_ANGLE_KP = new Supplier<Double>() {
  @Override
  public Double get() {
    return SmartDashboard.getNumber("kp", 0);
  }
};  
  public static final double SPIN_ANGLE_KI = 0;
  public static final Supplier<Double> SPIN_ANGLE_KD = new Supplier<Double>() {
    @Override
    public Double get() {
      return SmartDashboard.getNumber("kd", 0);
    }
  };  

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

    public static double clamp(double value, double min, double max){
      return Math.max(min, Math.min(max, value));
    }

   

}
