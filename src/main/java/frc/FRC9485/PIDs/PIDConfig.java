package frc.FRC9485.PIDs;

@SuppressWarnings("unused")
public class PIDConfig {
    
    private double Kp;
    private double Ki;
    private double Kd;
    private double Kf;
    private double iZone;

    public record PIDConfiguration(double kp, double ki, double kd, double kf, double iZone) {}
    private PIDConfiguration configuration;

    public PIDConfig(double Kp, double Ki, double Kd, double Kf, double iZone){
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.iZone = iZone;
        this.configuration = new PIDConfiguration(Kp, Ki, Kd, Kf, iZone);
    }

    public PIDConfig(double Kp, double Ki, double Kd, double Kf){
        this(Kp, Ki, Kd, Kf, 0);
    }

    public PIDConfig(double Kp, double Ki, double Kd){
        this(Kp, Ki, Kd, 0, 0);
    }
}
