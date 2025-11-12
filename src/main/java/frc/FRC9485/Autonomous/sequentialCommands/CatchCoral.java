package frc.FRC9485.Autonomous.sequentialCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.level.intake.SetIntakeSpeed;
import frc.robot.subsystems.Mechanism.SuperStructure;
import frc.robot.subsystems.Mechanism.SuperStructure.StatesToScore;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;

public class CatchCoral extends SequentialCommandGroup{

    IntakeSubsystem intakeSubsystem;
    SuperStructure superStructure;

    public CatchCoral(IntakeSubsystem intakeSubsystem, SuperStructure superStructure){
        this.intakeSubsystem = intakeSubsystem;
        this.superStructure = superStructure;

        addCommands(
            new ParallelCommandGroup(
                superStructure.scorePieceOnLevelAutonomous(StatesToScore.L1),
                new SetIntakeSpeed()
            )
        );
    }
}
