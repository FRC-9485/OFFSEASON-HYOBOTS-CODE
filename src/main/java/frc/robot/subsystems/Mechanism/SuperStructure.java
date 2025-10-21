package frc.robot.subsystems.Mechanism;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.ElevatorPositions;
import frc.robot.Constants.Intake.IntakePositions;
import frc.robot.subsystems.Led.LedSubsystem;
import frc.robot.subsystems.Mechanism.elevator.ElevatorSubsystem;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;

public class SuperStructure extends SubsystemBase{

    private ElevatorSubsystem elevatorSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private LedSubsystem ledSubsystem;

    private double elevatorInput;
    private double intakeInput;

    private String state;
    private Color color;
    private boolean isFinished;

    public static SuperStructure mInstance = null;

    private SuperStructure(){
        this.elevatorSubsystem = ElevatorSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.ledSubsystem = LedSubsystem.getInstance();

        this.elevatorInput = ElevatorPositions.HOME;
        this.intakeInput = IntakePositions.DEFAULT_POSITION;

        this.color = Color.kBlack;
        this.state = "INITIAL STATE";
        this.isFinished = intakeSubsystem.atSetpoint() && elevatorSubsystem.atSetpoint();
    }

    public static SuperStructure getInstance(){
        if(mInstance == null){
            mInstance = new SuperStructure();
        }
        return mInstance;
    }

    public String getState(){
        return state;
    }

    @Override
    public void periodic() {
        this.elevatorSubsystem.setPosition(elevatorInput);
        this.intakeSubsystem.setPosition(intakeInput);

        this.elevatorSubsystem.periodic();
        this.intakeSubsystem.periodic();
        alternLedColor(color);

        Logger.processInputs(state, intakeSubsystem);                               
    }

    public boolean scoreIsFinised(){
        return elevatorSubsystem.atSetpoint() && intakeSubsystem.atSetpoint();
    }
    
    public enum StatesToScore{
        L1(1),
        L2(2),
        L3(3),
        L4(4),

        ALGAE_L2(5),
        ALGAE_L3(6),
        PROCESSOR(7);

        int button;
        StatesToScore(int button){
            this.button = button;
        }

        public int getButtonPessed(){
            return button;
        }
    }

    public enum intakePositions{
        DEFAULT,
        CONTROL_BALL,
        SIMPLE_OPEN,
        PUT_CORAL,
        PUT_L4
    }

    private void alternLedColor(Color color){
    if(DriverStation.isTeleopEnabled()){    
            if(intakeSubsystem.IsTouched()){
                    ledSubsystem.setPattern(LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, color));
                } else if(elevatorSubsystem.getOutputInElevatorMotors()[0] != 0){
                    ledSubsystem.setPattern(LEDPattern.gradient(GradientType.kDiscontinuous, Color.kWhite, color));
                } else {
                    ledSubsystem.setSolidColor(color);
            }
        }
    }

    public Command setIntakePosition(intakePositions position){
        return run(() ->{
            switch (position) {
                case DEFAULT:
                    this.intakeInput = IntakePositions.DEFAULT_POSITION;
                    break;
            
                case CONTROL_BALL:
                    this.intakeInput = IntakePositions.CONTROL_BALL;
                    break;

                case PUT_CORAL:
                    this.intakeInput = IntakePositions.PUT_CORAL;
                    break;

                case SIMPLE_OPEN:
                    this.intakeInput = IntakePositions.ABERTURA_COMUMM;
                    break;

                case PUT_L4:
                    this.intakeInput = IntakePositions.OPEN_L4;
                    break;
            }
        });
    }

    public Command scorePieceOnLevel(StatesToScore state){
        return run(() ->{
            switch (state) {
                case L1:
                this.state = "L1 STATE";
                this.color = Color.kGreen;
                if(this.elevatorInput != ElevatorPositions.HOME) this.intakeInput = IntakePositions.ABERTURA_COMUMM;
                if(this.intakeSubsystem.atSetpoint()){
                    this.elevatorInput = ElevatorPositions.HOME;
                    if(this.elevatorSubsystem.atSetpoint() && intakeSubsystem.getSetpoint() == IntakePositions.ABERTURA_COMUMM){
                        this.intakeInput = IntakePositions.DEFAULT_POSITION;
                    }
                }
                break;
                
                case L2:
                    this.state = "L2 STATE";
                    this.color = Color.kYellow;
                    this.intakeInput = IntakePositions.PUT_CORAL_ALTERNATIVE;
                    if(this.intakeSubsystem.atSetpoint()){
                        this.elevatorInput = ElevatorPositions.L2;
                    }    
                    break;
    
                case L3:
                    this.state = "L3 STATE";
                    this.color = Color.kPurple;
                    this.intakeInput = IntakePositions.PUT_CORAL_ALTERNATIVE;
                    if(this.intakeSubsystem.atSetpoint()){
                        this.elevatorInput = ElevatorPositions.L3;
                    }    
                    break;
    
                case L4:
                    this.state = "L4 STATE";
                    this.color = Color.kDarkBlue;
                    if(this.elevatorInput != ElevatorPositions.L4) this.intakeInput = IntakePositions.PUT_CORAL;
                    if(this.intakeSubsystem.atSetpoint()){
                        this.elevatorInput = ElevatorPositions.L4;
                        if(this.elevatorSubsystem.atSetpoint() && this.intakeSubsystem.getSetpoint() == IntakePositions.PUT_CORAL){
                            this.intakeInput = IntakePositions.OPEN_L4;
                        }
                    }
                    break;
    
                case ALGAE_L2:
                    this.state = "L2 ALGAE";
                    intakeInput = IntakePositions.CONTROL_BALL;
                    elevatorInput = ElevatorPositions.ALGAE_L2;
                    break;
    
                case ALGAE_L3:
                    this.state = "L3 ALGAE";
                    intakeInput = IntakePositions.CONTROL_BALL;
                    elevatorInput = ElevatorPositions.ALGAE_L3;
                    break;
    
                case PROCESSOR:
                    this.state = "PROCESSADOR";
                    intakeInput = IntakePositions.CONTROL_BALL;
                    elevatorInput = ElevatorPositions.PROCESSOR;
                    break;
            }
        });
    }

    public boolean HasGamePieceOnIntake(){
        return intakeSubsystem.hasCoralOnIntake();
    }

    public boolean seguranceSystem(){
        return intakeInput > IntakePositions.DEFAULT_POSITION;
    }
}
