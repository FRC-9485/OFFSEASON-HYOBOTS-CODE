package frc.robot.commands.level.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism.elevator.ElevatorSubsystem;

public class ElevatorPosition extends Command{

    private ElevatorSubsystem elevator;
    private double setpoint;

    public ElevatorPosition(double setpoint){
        this.elevator = ElevatorSubsystem.getInstance();
        this.setpoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println("incializando o elevador");
    }

    @Override
    public void execute() {
        elevator.setPosition(setpoint);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopElevator();
    }
}
