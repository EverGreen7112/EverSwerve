package frc.robot.Utils;

public class Funcs {

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

    public static double convertRotationsToDegrees(double rotations){
        //convert rotations to degrees
        rotations *= 360;
       
        //convert from -180 - 180 to 0 - 360 
        if(rotations < 0){
            rotations += 360;
        }
        return rotations;
    }
}
