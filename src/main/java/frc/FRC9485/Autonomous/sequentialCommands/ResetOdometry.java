package frc.FRC9485.Autonomous.sequentialCommands;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ResetOdometry extends Command{
    
    private SwerveSubsystem swerveSubsystem;

    private Pose2d pose;
    private Pigeon2 pigeon;

    public ResetOdometry(){
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.pigeon = new Pigeon2(9);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            pigeon.reset();
        }
        pose = SwerveSubsystem.getInstance().getSidePose();
    }

    @Override
    public void execute() {
        swerveSubsystem.resetOdometry(pose);
    }

    @Override
    public boolean isFinished() {
        System.out.println("ACABOU DE RESETAR");
        return swerveSubsystem.getPose() == pose;
    }

    @Override
    public void end(boolean interrupted) {}
}