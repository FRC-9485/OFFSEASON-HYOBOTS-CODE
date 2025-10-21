package frc.robot.commands.swerveUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TurnRobot extends Command{
    
    private SwerveSubsystem swerve;
    private double angulo;

    private PIDController controller;

    private double atual;
    private double output;

    public TurnRobot(double angulo){
        this.swerve = SwerveSubsystem.getInstance();
        this.angulo = angulo;
        this.controller = new PIDController(0.01, 0, 0);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        controller.setTolerance(2.0);
    }

    @Override
    public void execute() {
        try{
            
        atual = swerve.getYaw();
        output = controller.calculate(atual, angulo);

        swerve.drive(new Translation2d(0, 0), output, true);

        if(swerve.getYaw() >= angulo){
            swerve.drive(new Translation2d(0, 0), 0, true);
            return;
        }

    } catch(Exception e){
        System.out.println("erro o mudar angulação: " + e.getMessage());
    }
}

    @Override
    public boolean isFinished() {
        return controller.atSetpoint() || atual >= angulo;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true);
    }
}
