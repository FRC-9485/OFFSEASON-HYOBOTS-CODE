package frc.FRC9485.utils;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.FRC9485.Autonomous.sequentialCommands.CatchCoral;
import frc.FRC9485.Autonomous.sequentialCommands.PutCoralOnL2;
import frc.FRC9485.Autonomous.sequentialCommands.PutCoralOnL3;
import frc.FRC9485.Autonomous.sequentialCommands.PutCoralOnL4;
import frc.FRC9485.Autonomous.sequentialCommands.ResetOdometry;
import frc.FRC9485.Autonomous.sequentialCommands.ThrowAndCatchCoral;
import frc.robot.commands.level.intake.SetIntakeSpeed;
import frc.robot.commands.swerveUtils.AlingToTarget;
import frc.robot.commands.swerveUtils.ResetPigeon;
import frc.robot.commands.swerveUtils.TurnRobot;
import frc.robot.subsystems.Mechanism.SuperStructure;
import frc.robot.subsystems.Mechanism.SuperStructure.StatesToScore;
import frc.robot.subsystems.Mechanism.elevator.ElevatorSubsystem;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightConfig;

public class RegisterNamedCommands {

    private SwerveSubsystem swerve;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private LimelightConfig limelightConfig;
    private SuperStructure struct;  

    public static RegisterNamedCommands mInstance = null;

    private RegisterNamedCommands(){
        this.swerve = SwerveSubsystem.getInstance();
        this.elevator = ElevatorSubsystem.getInstance();
        this.intake = IntakeSubsystem.getInstance();
        this.limelightConfig = LimelightConfig.getInstance();
        this.struct = SuperStructure.getInstance();
    }

    public static RegisterNamedCommands getInstance(){
        if(mInstance == null){
            mInstance = new RegisterNamedCommands();
        }
        return mInstance;
    }

    public void configureNamedCommands(){
        configurePositionsToAutonomous(struct);
        configureSubsystemsUtils(intake);
        configureSwerveAutoCommands(swerve, limelightConfig, elevator);
        configureScoreCommands(intake, struct);
    }

    private void configureScoreCommands(IntakeSubsystem intakeSubsystem, SuperStructure superStructure){
        NamedCommands.registerCommand("L2", new PutCoralOnL2(intakeSubsystem, superStructure));
        NamedCommands.registerCommand("L3", new PutCoralOnL3(intakeSubsystem, superStructure)); 
        NamedCommands.registerCommand("L4", new PutCoralOnL4(superStructure, intakeSubsystem));
        NamedCommands.registerCommand("STOP SWERVE", swerve.stopSwerve().until(() -> swerve.swerveIsStoped()));
        NamedCommands.registerCommand("PEGAR CORAL", new CatchCoral(intakeSubsystem, superStructure));
        NamedCommands.registerCommand("JOGAR CORAL", new ThrowAndCatchCoral(intakeSubsystem, superStructure));
    }

    private void configurePositionsToAutonomous(SuperStructure struct){

        NamedCommands.registerCommand("SCORE ON L1", struct.scorePieceOnLevel(StatesToScore.L1));
        NamedCommands.registerCommand("GO TO HOME POSITION", new ParallelCommandGroup(
            struct.scorePieceOnLevel(StatesToScore.L1),
            new SetIntakeSpeed()
        ));
        NamedCommands.registerCommand("SCORE ON L2", struct.scorePieceOnLevel(StatesToScore.L2));
        NamedCommands.registerCommand("SCORE ON L3", struct.scorePieceOnLevel(StatesToScore.L3));
        NamedCommands.registerCommand("SCORE ON L4", struct.scorePieceOnLevel(StatesToScore.L4));

        NamedCommands.registerCommand("SCORE ALGAE ON LEVEL 2", struct.scorePieceOnLevel(StatesToScore.ALGAE_L2));
        NamedCommands.registerCommand("SCORE ALGAE ON LEVEL 3", struct.scorePieceOnLevel(StatesToScore.ALGAE_L3));
        NamedCommands.registerCommand("SCORE ON PROCESSOR", struct.scorePieceOnLevel(StatesToScore.PROCESSOR));
    }

    private void configureSwerveAutoCommands(SwerveSubsystem swerve, LimelightConfig limelightConfig, ElevatorSubsystem elevatorSubsystem){

        NamedCommands.registerCommand("RESET PIGEON", new ResetOdometry());

        NamedCommands.registerCommand("ALINHAMENTO", new AlingToTarget());

        NamedCommands.registerCommand("TURN ROBOT", new TurnRobot(45));

        NamedCommands.registerCommand("RESET ELEVATOR", elevatorSubsystem.resetElevator().until(() -> elevatorSubsystem.getDistance() >= 0.0001));

        NamedCommands.registerCommand("RESET ODOMETRY", new InstantCommand(() ->{
            swerve.resetOdometry(new Pose2d(0.0, 0, new Rotation2d(180)));
        }));

        NamedCommands.registerCommand("RESET TESTE", swerve.runResetOdometry(new Pose2d(7.564, 3.851, new Rotation2d(180))));

        NamedCommands.registerCommand("TURN IN 0", swerve.putSwerveIn0().until(() -> swerve.swerveIsStoped()));
    }

    private void configureSubsystemsUtils(IntakeSubsystem intakeSubsystem){

        NamedCommands.registerCommand("THROW CORAL", new SetIntakeSpeed(0.8));

        NamedCommands.registerCommand("GET CORAL", new SetIntakeSpeed());
    }
}