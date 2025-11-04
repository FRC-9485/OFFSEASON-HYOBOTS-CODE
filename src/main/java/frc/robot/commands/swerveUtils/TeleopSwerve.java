package frc.robot.commands.swerveUtils;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TeleopSwerve extends Command{
    
    XboxController controller = new XboxController(0);

    DoubleSupplier x, y, rotation;

    double MAX_SPEED;

    SwerveSubsystem swerveSubsystem;

    double flipUtil = 1;

    public TeleopSwerve(XboxController controller){
        this.controller = controller;
        this.x = () -> controller.getLeftY() * flipUtil;
        this.y = () -> controller.getLeftX() * flipUtil;
        this.rotation = () -> controller.getLeftX() * flipUtil;
        this.swerveSubsystem = SwerveSubsystem.getInstance();

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.flipUtil = DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1;
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(new Translation2d(x.getAsDouble(), y.getAsDouble()), rotation.getAsDouble(), true);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
