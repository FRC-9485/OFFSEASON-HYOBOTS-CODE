package frc.FRC9485.utils;

public class Util {
    
    public static boolean inRange(double value, double min, double max){
        return max > value && min < value;
    }

    public static boolean inReference(double value, double setpoint){
        return value == setpoint;
    }

    public static double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
      }
    
}
