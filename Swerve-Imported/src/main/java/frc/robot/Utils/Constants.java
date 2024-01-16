package frc.robot.Utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface Constants {

    public class SwerveValues {

        public static final boolean USES_ABS_ENCODER = true;

        // chassis motor ports
        public static final int TOP_LEFT_DRIVE_PORT = 6;
        public static final int TOP_RIGHT_DRIVE_PORT = 8;
        public static final int DOWN_LEFT_DRIVE_PORT = 4;
        public static final int DOWN_RIGHT_DRIVE_PORT = 10;
        public static final int TOP_LEFT_STEERING_PORT = 7;
        public static final int TOP_RIGHT_STEERING_PORT = 9;
        public static final int DOWN_LEFT_STEERING_PORT = 5;
        public static final int DOWN_RIGHT_STEERING_PORT = 11;

        // chassis encoders
        public static final int TOP_LEFT_CANCODER = 1;
        public static final int TOP_RIGHT_CANCODER = 4;
        public static final int DOWN_LEFT_CANCODER = 5;
        public static final int DOWN_RIGHT_CANCODER = 0;

        public static final double TOP_RIGHT_CANCODER_OFFSET = 0.293212890625;
        public static final double TOP_LEFT_CANCODER_OFFSET = 0.291748046875;
        public static final double DOWN_RIGHT_CANCODER_OFFSET = -0.1171875;
        public static final double DOWN_LEFT_CANCODER_OFFSET = 0.310546875;

        // chassis size
        public static final double FRONT_WHEEL_DIST_METERS = 0.57;
        public static final double SIDE_WHEEL_DIST_METERS = 0.57;
        public static final double WHEEL_PERIMETER = Math.PI * 0.095;
        public static final double ROBOT_BOUNDING_CIRCLE_PERIMETER = Math.PI * Math.sqrt(
                FRONT_WHEEL_DIST_METERS * FRONT_WHEEL_DIST_METERS + SIDE_WHEEL_DIST_METERS * SIDE_WHEEL_DIST_METERS);

        // swerve vectors
        public static final Vector2d TOP_RIGHT = new Vector2d((FRONT_WHEEL_DIST_METERS / 2),
                (SIDE_WHEEL_DIST_METERS / 2));
        public static final Vector2d TOP_LEFT = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2),
                SIDE_WHEEL_DIST_METERS / 2);
        public static final Vector2d DOWN_RIGHT = new Vector2d(FRONT_WHEEL_DIST_METERS / 2,
                -(SIDE_WHEEL_DIST_METERS / 2));
        public static final Vector2d DOWN_LEFT = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2),
                -(SIDE_WHEEL_DIST_METERS / 2));

        // array of physical module vectors
        public static final Vector2d[] physicalMoudulesVector = { TOP_RIGHT, TOP_LEFT,
                DOWN_RIGHT, DOWN_LEFT };// array of vectors from robot center to swerves module

        // module gear ratios
        public static final double DRIVE_GEAR_RATIO = 1 / 6.75; // L2
        public static final double STEERING_GEAR_RATIO = 1 / 12.8;
    }

    public class SpeedValues {
        public static final double AUTONOMOUS_MAX_ANGULAR_SPEED = 0.5;

        // max speed values in m/s
        public static final Supplier<Double> MAX_SPEED = new Supplier<Double>() {
            @Override
            public Double get() {
                return SmartDashboard.getNumber("max drive speed", 1);
            }
        };

        // speed values in m/s
        public static final Supplier<Double> MAX_ANGULAR_SPEED = new Supplier<Double>() {
            @Override
            public Double get() {
                return SmartDashboard.getNumber("max angular speed", 1);
            }
        };

    }

    public class PIDValues {

        public static final double WHEEL_ANGLE_KP = 0.01;
        public static final double WHEEL_ANGLE_KI = 0.0;
        public static final double WHEEL_ANGLE_KD = 0.0;

        public static final double WHEEL_VELOCITY_KP = 0.05;
        public static final double WHEEL_VELOCITY_KI = 0;
        public static final double WHEEL_VELOCITY_KD = 0;
        public static final double WHEEL_VELOCITY_KF = 0.75 / 2.81;

        // pid on the pos on the x axis
        public static final double X_KP = 5;
        public static final double X_KI = 0;
        public static final double X_KD = 0;

        // pid on the pos on the y axis
        public static final double Y_KP = 5;
        public static final double Y_KI = 0;
        public static final double Y_KD = 0;

        public static final double X_THRESHOLD = 0.04;
        public static final double Y_THRESHOLD = 0.04;

        // speed values
        public static final double HEADING_KP = 0.03;
        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0;
        public static final double HEADING_TOLERANCE = 3;

    }

    public class JoystickValues {

        public static final int LEFT_JOYSTICK = 0;
        public static final int RIGHT_JOYSTICK = 1;
        public static final int OPERATOR = 2;
        public static final double JOYSTICK_DEADZONE = 0.2;

    }

}
