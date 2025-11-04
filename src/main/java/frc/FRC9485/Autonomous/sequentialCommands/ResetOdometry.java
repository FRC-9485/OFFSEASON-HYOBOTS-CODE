package frc.FRC9485.Autonomous.sequentialCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ResetOdometry extends Command{
    
    SwerveSubsystem swerveSubsystem;

    Pose2d pose;

    public ResetOdometry(Pose2d pose){
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.pose = pose;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.resetOdometry(pose);
    }

    @Override
    public boolean isFinished() {
        return swerveSubsystem.getPose() == pose;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
