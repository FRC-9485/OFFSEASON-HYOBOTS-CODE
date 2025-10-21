package frc.robot.commands.level.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;

public class GoToGetCoralPosition extends Command{
    
    IntakeSubsystem intakeSubsystem;

    public GoToGetCoralPosition(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        intakeSubsystem.goToGetCoralPosition();
        intakeSubsystem.getCoral();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.hasCoralOnIntake();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopCoralMotor();
    }
}
