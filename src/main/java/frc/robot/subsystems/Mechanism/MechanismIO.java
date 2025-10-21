package frc.robot.subsystems.Mechanism;

public interface MechanismIO {
    
    double getDistance();

    double getSetpoint();

    boolean atSetpoint();

    void setPosition(double setpoint);

    void setSpeed(double speed);
}
