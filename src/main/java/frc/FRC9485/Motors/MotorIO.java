package frc.FRC9485.Motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

public interface MotorIO {
    
    double getVoltage();

    double getPosition();

    double getSpeed();

    boolean atSetpoint(double setpoint);

    RelativeEncoder getAlternativeEncoder(boolean usingRelativeEncoder);

    AbsoluteEncoder getAbsoluteEncoder(boolean usingAbsoluteEncoder);

    void setSpeed(double speed);

    void setVoltage(double voltage);

    void FollowMotor(int id, boolean inverted);

    double getMotorTemperature();

    double getMotorOutput();

    SparkMax getSpark();

    int getMotorId();
    
    void setPosition(double position);

    String getMotorName();

    boolean usingInternalMotor();

    void setPID(double Kp, double Ki, double Kd);

    void clearStickyFaults();

    void setReferencePosition(double position);

    void setRampRate(double ramp);
}
