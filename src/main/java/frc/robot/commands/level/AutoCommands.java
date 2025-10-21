package frc.robot.commands.level;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism.SuperStructure;
import frc.robot.subsystems.Mechanism.SuperStructure.StatesToScore;

public class AutoCommands extends Command{

    SuperStructure superStructure;
    StatesToScore statesToScore;

    public AutoCommands(StatesToScore statesToScore){
        this.superStructure = SuperStructure.getInstance();
        this.statesToScore = statesToScore;
        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        superStructure.scorePieceOnLevel(statesToScore);
    }

    @Override
    public boolean isFinished() {
        return superStructure.scoreIsFinised();
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}