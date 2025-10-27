// package frc.FRC9485.Autonomous.sequentialCommands;

// import com.ctre.phoenix6.hardware.CANcoder;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.subsystems.swerve.SwerveSubsystem;

// public class StopSwerveDrive extends Command{
    
//     SwerveSubsystem subsystem;

//     CANcoder encoder;

//     public StopSwerveDrive(){
//         this.subsystem = SwerveSubsystem.getInstance();
//         this.encoder = new CANcoder(11);

//         addRequirements(subsystem);
//     }

//     @Override
//     public void initSendable(SendableBuilder builder) {}

//     @Override
//     public void execute() {
//         if(DriverStation.isAutonomous()){
//             subsystem.drive(new Translation2d(0, 0), 0, true);
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return encoder.getVelocity().getValueAsDouble() == 0;
//     }

//     @Override
//     public void end(boolean interrupted) {
        
//     }
// }
