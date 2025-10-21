package frc.FRC9485.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.FRC9485.utils.Util;
import frc.robot.Constants.Controllers;
import frc.robot.Constants.Positions;

public class MechanismJoystick implements IDMechanism{
    
    private CommandXboxController controller;
    public double setpoint;

    public static MechanismJoystick mInstance = null;

    private MechanismJoystick(){
        this.controller = new CommandXboxController(Controllers.INTAKE_CONTROLLER);
    }
    
    public static MechanismJoystick getInstance(){
        if(mInstance == null){
            mInstance =  new MechanismJoystick();
        }
        return mInstance;
    }

    @Override
    public Trigger getCoralButton(){
        return controller.povDown();
    }

    @Override
    public XboxController getHID(){
        return controller.getHID();
    }

    @Override
    public Trigger throwCoralOnIntake(){
        return controller.povCenter();
    }

    @Override
    public Trigger L1Button(){
        this.setpoint = Positions.L1_POSITION;
        return controller.a();
    }

    @Override
    public Trigger L2Button(){
        this.setpoint = Positions.L2_POSITION;
        return controller.b();
    }

    @Override
    public Trigger L3Button(){
        this.setpoint = Positions.L3_POSITION;
        return controller.y();
    }

    @Override
    public Trigger L4Button(){
        this.setpoint = Positions.L4_POSITION;
        return controller.x();
    }

    @Override
    public Trigger ProcessorButton(){
        return L2Button().and(L3Algae());
    }

    @Override
    public Trigger L2Algae(){
        this.setpoint = Positions.ALGAE_L2;
        return controller.rightBumper();
    }

    @Override
    public Trigger L3Algae(){
        this.setpoint = Positions.ALGAE_L3;
        return controller.leftBumper();
    }

    @Override
    public double UpElevator() {
       return controller.getRightY();
    }

    @Override
    public double getSetpoint(){
        return this.setpoint;
    }

    @Override
    public boolean joystickIsNothingUsingMechanism() {
        return Util.inRange(controller.getRightY(), -Controllers.DEADBAND, Controllers.DEADBAND) 
        && Util.inRange(controller.getRightX(), -Controllers.DEADBAND, Controllers.DEADBAND);
    }

    @Override
    public double getRightTrigger(){
        return controller.getRightTriggerAxis();
    }

    @Override
    public double getLeftTrigger(){
        return controller.getLeftTriggerAxis();
    }
}
