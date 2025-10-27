package frc.FRC9485.Autonomous.sequentialCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.level.intake.SetIntakeSpeed;
import frc.robot.subsystems.Mechanism.SuperStructure;
import frc.robot.subsystems.Mechanism.SuperStructure.StatesToScore;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;

public class ThrowAndCatchCoral extends SequentialCommandGroup{

    IntakeSubsystem intakeSubsystem;
    SuperStructure superStructure;

    public ThrowAndCatchCoral(IntakeSubsystem intakeSubsystem, SuperStructure superStructure){
        this.intakeSubsystem = intakeSubsystem;
        this.superStructure = superStructure;

        addCommands(
            new SetIntakeSpeed(0.5).until(() -> !intakeSubsystem.IsTouched()),
            new ParallelCommandGroup(
                new SetIntakeSpeed().onlyIf(() -> !intakeSubsystem.IsTouched()),
                superStructure.scorePieceOnLevel(StatesToScore.L1)
            )
        );
    }
}