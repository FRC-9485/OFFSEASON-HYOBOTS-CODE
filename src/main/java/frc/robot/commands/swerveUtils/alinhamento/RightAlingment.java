package frc.robot.commands.swerveUtils.alinhamento;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers;

public class RightAlingment extends Command{
    
    private SwerveSubsystem swerveSubsystem;
    private PIDController X, Y, rotation;
    private Timer dontSeeTag, limit;

    private double[] position;

    public RightAlingment(){
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.X = new PIDController(0.1, 0.1, 0.001);
        this.Y = new PIDController(0.04, 0, 0.03);
        this.rotation = new PIDController(0.31, 0, 0.00);

        this.limit = new Timer();
        this.dontSeeTag = new Timer();

        addRequirements(swerveSubsystem);

        SmartDashboard.putData("X pid", X);
        SmartDashboard.putData("Y pid", Y);
        SmartDashboard.putData("Rotation pid", rotation);

        // position[0] = X do robô em relação ao alvo (metros)
        // position[1] = Y do robô em relação ao alvo (metros)
        // position[2] = Z (altura)
        // position[3] = rotação roll
        // position[4] = pitch
        // position[5] = yaw (ângulo para o alvo) -> isso é target_space
    }

    @Override
    public void initialize() {
        this.position = LimelightHelpers.getTargetPose_RobotSpace("");
        
        //Todo que estão comentados devem ser mudados para os valores respectivos
        this.rotation.reset();
        this.rotation.setTolerance(0.05);
        this.rotation.enableContinuousInput(-180, 180);
        this.rotation.setSetpoint(Math.abs(position[5])); // alinhar reto com a tag
        
        this.X.reset();
        this.X.setTolerance(0.1);
        this.X.setSetpoint(Math.abs(position[0])); //alinhar no centro da tag

        this.Y.reset();        
        this.Y.setTolerance(0.1);
        this.Y.setSetpoint(Math.abs(position[1])); // alinhar distancia de lado -> direita significa que ele deve ir um pouco mais ao lado

        this.limit.reset();
        this.limit.start();
    }
    
    @Override
    public void execute() {
        if(LimelightHelpers.getTV("")){

            // this.position = LimelightHelpers.getBotPose_TargetSpace(""); // pose do robo em metros
            this.dontSeeTag.reset();

            double normalize = LimelightHelpers.getBotPose_TargetSpace("")[5];

            double Xcontroller = X.calculate(LimelightHelpers.getTX(""));
            double Ycontroller = Y.calculate(LimelightHelpers.getTY(""));
            double omega = rotation.calculate(normalize); // a pose é em metros, já o alinhamento é em centimetros
            
            if(!Double.isNaN(Xcontroller) && !Double.isNaN(Ycontroller) && !Double.isNaN(omega)){
                swerveSubsystem.drive(new Translation2d(-Xcontroller, Ycontroller), omega, true);
            }
         }else {
            this.cancel();
            DriverStation.reportError("O alinhamento não pode ser concluido devido a ausencia da tag", false);
        }
    }

    @Override
    public boolean isFinished() {
        return X.atSetpoint() && Y.atSetpoint() && rotation.atSetpoint() ||
        dontSeeTag.hasElapsed(4) ||
        limit.hasElapsed(10);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
    }
}