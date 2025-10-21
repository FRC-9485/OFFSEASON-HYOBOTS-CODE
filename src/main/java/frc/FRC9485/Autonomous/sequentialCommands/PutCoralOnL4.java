package frc.FRC9485.Autonomous.sequentialCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.level.intake.SetIntakeSpeed;
import frc.robot.subsystems.Mechanism.SuperStructure;
import frc.robot.subsystems.Mechanism.SuperStructure.StatesToScore;
import frc.robot.subsystems.Mechanism.SuperStructure.intakePositions;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;

public class PutCoralOnL4 extends SequentialCommandGroup{
    
    SuperStructure superStructure;
    IntakeSubsystem intakeSubsystem;

    public PutCoralOnL4(SuperStructure superStructure, IntakeSubsystem intakeSubsystem){
        this.superStructure = superStructure;
        this.intakeSubsystem = intakeSubsystem;

        addCommands(
            superStructure.scorePieceOnLevel(StatesToScore.L4).until(() -> superStructure.scoreIsFinised()),
            new SetIntakeSpeed(0.5).onlyWhile(() -> intakeSubsystem.IsTouched()),
            superStructure.setIntakePosition(intakePositions.PUT_CORAL),
            new ParallelCommandGroup(
                new SetIntakeSpeed(),
                superStructure.scorePieceOnLevel(StatesToScore.L1)
            )
        );
    }
}
