package frc.robot.Utils;

public class Consts {
    //chassis motors
    public static final int TOP_LEFT_SPEED_PORT = 3;
    public static final int TOP_RIGHT_SPEED_PORT = 1;
    public static final int DOWN_LEFT_SPEED_PORT = 5;
    public static final int DOWN_RIGHT_SPEED_PORT = 7;
    public static final int TOP_LEFT_ROT_PORT = 4;
    public static final int TOP_RIGHT_ROT_PORT = 2;
    public static final int DOWN_LEFT_ROT_PORT = 6;
    public static final int DOWN_RIGHT_ROT_PORT = 8;

    //chassis encoders
    public static final int TOP_LEFT_CANCODER = 11;
    public static final int TOP_RIGHT_CANCODER = 10;
    public static final int DOWN_LEFT_CANCODER = 12;
    public static final int DOWN_RIGHT_CANCODER = 13;

    //pid values
    public static final double WHEEL_ROTATION_KP = 0.0124; 
    public static final double WHEEL_ROTATION_KI = 0;
    public static final double WHEEL_ROTATION_KD = 0;


    //swerve vectors
    public static final Vector2d TOP_RIGHT = new Vector2d(0.2551, 0.31535);
    public static final Vector2d TOP_LEFT = new Vector2d(-0.2551, 0.31535);
    public static final Vector2d DOWN_RIGHT = new Vector2d(0.2551, -0.31535);
    public static final Vector2d DOWN_LEFT = new Vector2d(-0.2551, -0.31535);

    public static final Vector2d[] physicalMoudulesVector = { TOP_LEFT, TOP_RIGHT, DOWN_LEFT, DOWN_RIGHT};//array of vectors from robot center to swerves module

    public static final double GEAR_RATIO = 8.14;


    public static double rpmToMs(double wheelRadius, double rpm){
      double rps = rpm / 60;
      return wheelRadius * 2 * Math.PI * rps;
    }

    public static double rotationsToDegrees(double rotations){
      return (rotations / GEAR_RATIO) / 360;
    }

    public static double degreesToRotations(double angle){
      return (angle / 360) * GEAR_RATIO;
    }
}
