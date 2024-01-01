package frc.robot.Utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Swerve;

public class Consts {

  public static final boolean USES_ABS_ENCODER = false;

  // max speed values in m/s
  public static final Supplier<Double> MAX_SPEED = new Supplier<Double>() {
    @Override
    public Double get() {
      return SmartDashboard.getNumber("max drive speed", 2);
    }
  };

  // speed values in m/s
  public static final Supplier<Double> MAX_ANGULAR_SPEED = new Supplier<Double>() {
    @Override
    public Double get() {
      return SmartDashboard.getNumber("max angular speed", 5);
    }
  };

  // chassis motors
  public static final int TOP_LEFT_DRIVE_PORT = 6;
  public static final int TOP_RIGHT_DRIVE_PORT = 8;
  public static final int DOWN_LEFT_DRIVE_PORT = 4;
  public static final int DOWN_RIGHT_DRIVE_PORT = 10;
  public static final int TOP_LEFT_STEERING_PORT = 7;
  public static final int TOP_RIGHT_STEERING_PORT = 9;
  public static final int DOWN_LEFT_STEERING_PORT = 5;
  public static final int DOWN_RIGHT_STEERING_PORT = 11;

  // chassis encoders
  public static final int TOP_LEFT_CANCODER = 0;
  public static final int TOP_RIGHT_CANCODER = 2;
  public static final int DOWN_LEFT_CANCODER = 1;
  public static final int DOWN_RIGHT_CANCODER = 3;

  // cancoder offset(remember to update with robot!!)
  // remember to sub your offset from 360
  public static final double TOP_RIGHT_CANCODER_OFFSET = 360 - 126.9140625; // 360 - 4.218
  public static final double TOP_LEFT_CANCODER_OFFSET = 360 - 116.54296875;// 360 - 247.24
  public static final double DOWN_RIGHT_CANCODER_OFFSET = 360 - 18.80859375; // 360 - 72.246
  public static final double DOWN_LEFT_CANCODER_OFFSET = 360 - 250.6640625;// 360 - 238.71

  // joysticks
  public static final int LEFT_JOYSTICK = 0;
  public static final int RIGHT_JOYSTICK = 1;
  public static final double JOYSTICK_DEADZONE = 0.2;

  // pid values
  public static final double WHEEL_ANGLE_KP = 0.01; // 0.08
  public static final double WHEEL_ANGLE_KI = 0.0;// 0.000007
  public static final double WHEEL_ANGLE_KD = 0.0;

  public static final double WHEEL_VELOCITY_KP = 0.05;
  public static final double WHEEL_VELOCITY_KI = 0;
  public static final double WHEEL_VELOCITY_KD = 0;
  public static final double WHEEL_VELOCITY_KF = 0.75 / 2.81;

  // pid on the pos on the x axis
  public static final double X_KP = 1.5;
  public static final double X_KI = 0;
  public static final double X_KD = 0;

  // pid on the pos on the y axis
  public static final double Y_KP = 1.5;
  public static final double Y_KI = 0;
  public static final double Y_KD = 0;

  public static final double X_THRESHOLD = 0.04;
  public static final double Y_THRESHOLD =  0.04;

  // chassis size
  public static final double FRONT_WHEEL_DIST_METERS = 0.57;
  public static final double SIDE_WHEEL_DIST_METERS = 0.57;
  public static final double WHEEL_PERIMETER = Math.PI * 0.095;
  public static final double ROBOT_BOUNDING_CIRCLE_PERIMETER = Math.PI * Math.sqrt(FRONT_WHEEL_DIST_METERS * FRONT_WHEEL_DIST_METERS + SIDE_WHEEL_DIST_METERS * SIDE_WHEEL_DIST_METERS);

  // swerve vectors
  public static final Vector2d TOP_RIGHT = new Vector2d((FRONT_WHEEL_DIST_METERS / 2), (SIDE_WHEEL_DIST_METERS / 2));
  public static final Vector2d TOP_LEFT = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2), SIDE_WHEEL_DIST_METERS / 2);
  public static final Vector2d DOWN_RIGHT = new Vector2d(FRONT_WHEEL_DIST_METERS / 2, -(SIDE_WHEEL_DIST_METERS / 2));
  public static final Vector2d DOWN_LEFT = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2), -(SIDE_WHEEL_DIST_METERS / 2));

  // array of physical module vectors
  public static final Vector2d[] physicalMoudulesVector = { TOP_RIGHT, TOP_LEFT, DOWN_RIGHT, DOWN_LEFT };// array of
                                                                                                         // vectors from
                                                                                                         // robot center
                                                                                                         // to swerves
                                                                                                         // module

  // module gear ratios
  public static final double DRIVE_GEAR_RATIO = 1 / 6.75; // L2
  public static final double STEERING_GEAR_RATIO = 1 / 12.8;

  // speed values
  public static final double HEADING_KP = 0.03;
  public static final double HEADING_KI = 0;
  public static final double HEADING_KD = 0.0001;
  public static final double HEADING_TOLERANCE = 3;

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

  public static double roundAfterDecimalPoint(double num, int amount) {
    num *= Math.pow(10, amount);
    num = (int) num;
    num /= Math.pow(10, amount);
    return num;
  }

}
