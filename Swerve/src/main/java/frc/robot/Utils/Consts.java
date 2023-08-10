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
    public static final int TOP_LEFT_CANCODER = 0;
    public static final int TOP_RIGHT_CANCODER = 0;
    public static final int DOWN_LEFT_CANCODER = 0;
    public static final int DOWN_RIGHT_CANCODER = 0;

    //swerve vectors
    public static final Vector2d TOP_RIGHT = new Vector2d(0.2551, 0.31535);
    public static final Vector2d TOP_LEFT = new Vector2d(-0.2551, 0.31535);
    public static final Vector2d DOWN_RIGHT = new Vector2d(0.2551, -0.31535);
    public static final Vector2d DOWN_LEFT = new Vector2d(-0.2551, -0.31535);

    public static final Vector2d[] physicalMoudulesVector = { TOP_LEFT, TOP_RIGHT, DOWN_LEFT, DOWN_RIGHT};//array of vectors from robot center to swerves module

    public static final double ticksPerRev = 4096 * 8.14;//!!!update by gear ratio!!!
    public static final double wheelRadius = 0.05; //in meters

    //TPR is ticks per revolution
    public static double ticksToAngle(double ticks, double TPR){
      return (ticks * 360.0)/ TPR;
    }

    public static double rpmToMs(double wheelRadius, double rpm){
      double rps = rpm / 60;
      return wheelRadius * 2 * Math.PI * rps;
    }

    //TPR is ticks per revolution
    public static double angleToTicks(double angle, double TPR){
      return TPR / (360.0 / angle);
    }
}
