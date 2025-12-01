package frc.robot.commands.swerveUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers;

public class TestAlingment extends Command{
    
    private SwerveSubsystem swerveSubsystem;

    private PIDController X, Y, rotation;

    private Timer dontSeeTag, alingmentTime;

    private double tag = -1;

    public TestAlingment(){
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.X = new PIDController(0.04, 0.01, 0.01);
        this.Y = new PIDController(0.04, 0, 0.03);
        this.rotation = new PIDController(0.022, 0, 0.003);
        
        this.dontSeeTag = new Timer();
        this.alingmentTime = new Timer();

        SmartDashboard.putData("/Alinhamento/PID X", X);
        SmartDashboard.putData("/Alinhamento/PID Y" , Y);
        SmartDashboard.putData("/Alinhamento/PID Rotation", rotation);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

        this.X.setTolerance(-1.38);
        this.X.setSetpoint(1);
        
        this.Y.setSetpoint(1);
        this.Y.setTolerance(-0.436);

        this.rotation.setTolerance(0.1);
        this.rotation.setSetpoint(1);
        this.rotation.enableContinuousInput(-180, 180);

        this.alingmentTime.reset();
        this.alingmentTime.start();

        if(LimelightHelpers.getTV("")){
            this.tag = LimelightHelpers.getFiducialID("");
        }
    }

    @Override
    public void execute() {
        if(LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") ==  tag){
            this.dontSeeTag.reset();
            this.dontSeeTag.start();

            double[] position = LimelightHelpers.getBotPose_TargetSpace("");

            double entradaX = -X.calculate(position[2]);
            double entradaY = -Y.calculate(position[0]);
            double rot = rotation.calculate(position[4]);

            swerveSubsystem.drive(new Translation2d(entradaX, entradaY), rot, false);
            
            if(!rotation.atSetpoint() ||
               !X.atSetpoint() ||
               !Y.atSetpoint()){
                this.alingmentTime.reset();
            }
        } else{
            swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        return this.dontSeeTag.hasElapsed(10) || this.alingmentTime.hasElapsed(10) ||
        X.atSetpoint() && Y.atSetpoint() && rotation.atSetpoint();
    }
}
