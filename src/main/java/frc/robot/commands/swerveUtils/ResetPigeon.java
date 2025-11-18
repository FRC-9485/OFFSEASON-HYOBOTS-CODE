package frc.robot.commands.swerveUtils;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Components;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ResetPigeon extends Command{
    
    private Pigeon2 pigeon2;
    private SwerveSubsystem subsystem;

    public ResetPigeon(){
        this.subsystem = SwerveSubsystem.getInstance();
        this.pigeon2 = new Pigeon2(Components.PIGEON);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        try{

            pigeon2.reset();
            
        } catch(Exception e){
            System.out.println("erro ao resetar o pigeon");
        }
    }

    @Override
    public boolean isFinished() {
        return pigeon2.getYaw().getValueAsDouble() == 0.0;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.drive(new Translation2d(0, 0), 0, true);
    }
}
