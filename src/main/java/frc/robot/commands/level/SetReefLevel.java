package frc.robot.commands.level;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC9485.controllers.MechanismJoystick;
import frc.robot.Constants.Positions;
import frc.robot.Constants.Elevator.ElevatorPositions;
import frc.robot.Constants.Intake.IntakePositions;
import frc.robot.commands.level.elevator.ElevatorPosition;
import frc.robot.commands.level.intake.IntakePosition;
import frc.robot.subsystems.Mechanism.elevator.ElevatorSubsystem;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;

public class SetReefLevel extends Command{

    ElevatorSubsystem elevatorSubsystem;
    IntakeSubsystem intakeSubsystem;
    MechanismJoystick intakeController;

    double setpoint;
    boolean autonomous;

    private SequentialCommandGroup alternateLevel(double setpoint){
        SequentialCommandGroup sequentialCommandGroup = null;

        switch ((int) setpoint) {

            case (int) Positions.L1_POSITION:

                    sequentialCommandGroup = new SequentialCommandGroup(
                        new IntakePosition(IntakePositions.PUT_CORAL),
                        new ElevatorPosition(ElevatorPositions.HOME),
                        new IntakePosition(IntakePositions.DEFAULT_POSITION)
                    );


            case (int) Positions.L2_POSITION:
            
                    sequentialCommandGroup = new SequentialCommandGroup(
                        new IntakePosition(IntakePositions.PUT_CORAL),
                        new ElevatorPosition(ElevatorPositions.L2)
                    );

                
            case (int) Positions.L3_POSITION:
            
                    sequentialCommandGroup = new SequentialCommandGroup(
                        new IntakePosition(IntakePositions.PUT_CORAL),
                        new ElevatorPosition(ElevatorPositions.L3)
                    );

                
            case (int) Positions.L4_POSITION:
            
                    sequentialCommandGroup = new SequentialCommandGroup(
                        new IntakePosition(IntakePositions.PUT_CORAL),
                        new ElevatorPosition(ElevatorPositions.L4),
                        new IntakePosition(IntakePositions.OPEN_L4)
                    );

            case (int) Positions.PROCESSADOR:

                    sequentialCommandGroup = new SequentialCommandGroup(
                        new IntakePosition(IntakePositions.CONTROL_BALL),
                        new ElevatorPosition(ElevatorPositions.HOME)
                    );

            case (int) Positions.ALGAE_L2:

                    sequentialCommandGroup = new SequentialCommandGroup(
                        new IntakePosition(IntakePositions.CONTROL_BALL),
                        new ElevatorPosition(ElevatorPositions.ALGAE_L2)
                    );

            case (int) Positions.ALGAE_L3:

                    sequentialCommandGroup = new SequentialCommandGroup(
                        new IntakePosition(IntakePositions.CONTROL_BALL),
                        new ElevatorPosition(ElevatorPositions.ALGAE_L3)
                    );
        }
        
        return sequentialCommandGroup;
    }

    public SetReefLevel(){
        this.setpoint = intakeController.getSetpoint();
        this.intakeController = MechanismJoystick.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.elevatorSubsystem = ElevatorSubsystem.getInstance();
        addRequirements(intakeSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        alternateLevel(setpoint);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atSetpoint() && intakeSubsystem.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopElevator();
        intakeSubsystem.stopIntakeMotor();
    }
}
