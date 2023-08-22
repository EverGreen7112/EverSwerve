package frc.robot.Utils;

public class Consts {

    //speed values
    public static final double MAX_SPEED = 0.45;

    //chassis motors
    public static final int TOP_LEFT_SPEED_PORT = 15;
    public static final int TOP_RIGHT_SPEED_PORT = 16;
    public static final int DOWN_LEFT_SPEED_PORT = 17;
    public static final int DOWN_RIGHT_SPEED_PORT = 18;
    public static final int TOP_LEFT_ROT_PORT = 5;
    public static final int TOP_RIGHT_ROT_PORT = 7;
    public static final int DOWN_LEFT_ROT_PORT = 9;
    public static final int DOWN_RIGHT_ROT_PORT = 11;

    //chassis encoders
    public static final int TOP_LEFT_CANCODER = 6;
    public static final int TOP_RIGHT_CANCODER = 8;
    public static final int DOWN_LEFT_CANCODER = 10;
    public static final int DOWN_RIGHT_CANCODER = 12;

    public static final double TOP_RIGHT_CANCODER_OFFSET = 0.51;
    public static final double TOP_LEFT_CANCODER_OFFSET = 40.5;
    public static final double DOWN_RIGHT_CANCODER_OFFSET = 265.3;
    public static final double DOWN_LEFT_CANCODER_OFFSET = -133;

    //joysticks
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final double JOYSTICK_DEADZONE = 0.2;

    //pid values
    public static final double WHEEL_ROTATION_KP = 0.75; 
    public static final double WHEEL_ROTATION_KI = 0;
    public static final double WHEEL_ROTATION_KD = 0;

    //swerve vectors
    public static final Vector2d TOP_RIGHT = new Vector2d(0.29515, 0.29515); 
    public static final Vector2d TOP_LEFT = new Vector2d(-0.29515, 0.29515);
    public static final Vector2d DOWN_RIGHT = new Vector2d(0.29515, -0.29515);
    public static final Vector2d DOWN_LEFT = new Vector2d(-0.29515, -0.29515);

    public static final Vector2d[] physicalMoudulesVector = { TOP_RIGHT, TOP_LEFT, DOWN_RIGHT, DOWN_LEFT};//array of vectors from robot center to swerves module

    public static final double DRIVE_GEAR_RATIO = 8.14; //L1
    public static final double ROTATION_GEAR_RATIO = 12.8;


    public static double rpmToMs(double wheelRadius, double rpm){
      double rps = rpm / 60;
      return wheelRadius * 2 * Math.PI * rps;
    }

    public static double rotationsToDegrees(double rotations){
      return rotations * 360;
    }

    public static double degreesToRotations(double angle){
      return angle / 360;
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
