package frc.FRC9485.utils;

public class Util {
    
    public static boolean inRange(double value, double min, double max){
        return max > value && min < value;
    }

    public static boolean inReference(double value, double setpoint){
        return value == setpoint;
    }
}
