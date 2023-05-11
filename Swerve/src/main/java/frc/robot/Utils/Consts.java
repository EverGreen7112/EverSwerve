package frc.robot.Utils;

public class Consts {
    public static final Vector2d TOP_RIGHT = new Vector2d(1, 1);
    public static final Vector2d TOP_LEFT = new Vector2d(-1, 1);
    public static final Vector2d DOWN_RIGHT = new Vector2d(1, -1);
    public static final Vector2d DOWN_LEFT = new Vector2d(-1, -1);

    public static final Vector2d[] physicalMoudulesVector = { TOP_RIGHT, TOP_LEFT, DOWN_RIGHT, DOWN_LEFT };//array of vectors from robot center to swerves module

}
