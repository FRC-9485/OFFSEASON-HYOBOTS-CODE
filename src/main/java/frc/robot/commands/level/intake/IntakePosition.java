package frc.robot.commands.level.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;

public class IntakePosition extends Command{
    
    private IntakeSubsystem intake;
    private double setpoint;

    public IntakePosition(double setpoint){
        this.intake = IntakeSubsystem.getInstance();
        this.setpoint = setpoint;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("inicializando mudança de posição do intake");
    }

    @Override
    public void execute() {
        try{
        intake.setPosition(setpoint);
    } catch(Exception e){
        System.out.println("erro ao posicionar o intake: "+ e.getMessage());
    }
}
    @Override
    public boolean isFinished() {
        return intake.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntakeMotor();
    }
}
