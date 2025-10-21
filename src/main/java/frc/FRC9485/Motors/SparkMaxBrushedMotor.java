package frc.FRC9485.Motors;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import java.util.function.Supplier;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class SparkMaxBrushedMotor implements MotorIO{
    
    private int id;
    private boolean usingInternalEncoder;
    private String name;

    private SparkMaxConfig config;

    private SparkMax motor;

    private PIDController controller;
    private PIDConstants constants;

    private double percentOutput = 0;

    public SparkMaxBrushedMotor(int id, boolean usingInternalEncoder, String name){
        this.name = name;
        this.id = id;
        this.usingInternalEncoder = usingInternalEncoder;
        this.config = new SparkMaxConfig();
        this.constants = new PIDConstants(0, 0, 0);
        this.controller = new PIDController(constants.kP, constants.kI, constants.kD);
        this.motor = new SparkMax(id, SparkMax.MotorType.kBrushed);
        clearStickyFaults();
    }

    private void configureSpark(Supplier<REVLibError> config){
        for(int i = 0; i < 5; i++){
            if(config.get() == REVLibError.kError){
                return;
            }
            Timer.delay(Milliseconds.of(5).in(Seconds));
        }

        DriverStation.reportError("falha no motor: " + getMotorId(), true);
    }

    @Override
    public boolean atSetpoint(double setpoint){
        return Math.abs(getPosition() - setpoint) <= 0.001;
    }

    @Override
    public double getPosition(){
        return motor.getEncoder().getPosition();
    }

    @Override
    public void setReferencePosition(double position){
        if(!atSetpoint(position)){
            motor.getClosedLoopController().setReference(position, ControlType.kPosition);
        }
    }

    @Override
    public void clearStickyFaults(){
        configureSpark(motor::clearFaults);
    }

    public void updateConfig(SparkMaxConfig config){
        if(DriverStation.isEnabled()){
            throw new RuntimeException("o motor não pode ser fazer o update da sua configuração com o robo ligado");
        }

        this.config.apply(config);
        configureSpark(() -> motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public double getVoltage() {
       return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public double getSpeed() {
        return motor.get();
    }

    @Override
    public RelativeEncoder getAlternativeEncoder(boolean usingRelativeEncoder) {
        if(usingRelativeEncoder){
            return motor.getAlternateEncoder();
        } else{
        return null;
    }
    }

    @Override
    public boolean usingInternalMotor() {
        return usingInternalEncoder;
    }
    
    @Override
    public AbsoluteEncoder getAbsoluteEncoder(boolean usingAbsoluteEncoder) {
       if(usingAbsoluteEncoder){
        return motor.getAbsoluteEncoder();
       } else{
       return null;
    }
    }

    @Override
    public String getMotorName() {
        return name;
    }

    @Override
    public void setSpeed(double speed) {
        if(speed != percentOutput){
            motor.getClosedLoopController().setReference(speed, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0, 0);
        }

        this.percentOutput = speed;
    }

    @Override
    public void setPosition(double position){
        motor.getEncoder().setPosition(position);
    }

    @Override
    public int getMotorId() {
        return id;
    }

    @Override
    public void setVoltage(double voltage) {
       motor.setVoltage(voltage);
    }

    @Override
    public void FollowMotor(int id, boolean inverted) {
    config.follow(id);
    config.inverted(inverted);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public double getMotorTemperature() {
       return motor.getMotorTemperature();
    }

    @Override
    public double getMotorOutput(){
        return motor.getAppliedOutput();
    }

    @Override
    public SparkMax getSpark(){
        return motor;
    }

    @Override
    public void setPID(double Kp, double Ki, double Kd){
        controller.setP(Kp);
        controller.setI(Ki);
        controller.setD(Kd);
    }

    @Override
    public void setRampRate(double ramp){
        config.openLoopRampRate(ramp)
        .closedLoopRampRate(ramp);

        if(DriverStation.isEnabled()){
            System.out.println("ERRO! a rampa não pode ser mudada como robo ligado");
            return;
        }
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
}
