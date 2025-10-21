package frc.robot.commands.level.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;

public class SetIntakeSpeed extends Command{
    
    private IntakeSubsystem intake;
    private double speed;
    private boolean getCoralBoolean;

    public SetIntakeSpeed(double speed){
        this(speed, false);
    }

    public SetIntakeSpeed(){
        this(0.3, true);
    }

    private SetIntakeSpeed(double speed, boolean getCoralBoolean){
        this.intake = IntakeSubsystem.getInstance();
        this.speed = speed;
        this.getCoralBoolean = getCoralBoolean;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("velocidade do motor: " + speed);
        System.out.println("incializando mudança de velocidade");
    }

    @Override
    public void execute() {
        try{
            
        intake.setSpeed(speed);

    } catch(Exception e){
        System.out.println("erro ao colocar velocidade");
    }
}
    @Override
    public boolean isFinished() {
        return false || (getCoralBoolean == true && intake.IsTouched() == true);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopCoralMotor();
    }
}
