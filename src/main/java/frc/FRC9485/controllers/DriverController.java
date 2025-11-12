package frc.FRC9485.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.FRC9485.utils.Util;
import frc.robot.Constants.Controllers;

public class DriverController implements IDDriverController{

    public static DriverController mInstance = null;

    private CommandXboxController controller;
    private double invert;

    private DriverController(){
        this.controller = new CommandXboxController(Controllers.DRIVE_CONTROLLER);
        invert = 1;
    }
    
    public static DriverController getInstance(){
        if(mInstance == null){
            mInstance = new DriverController();
        }
        return mInstance;
    }

    @Override
    public double Invert(){
        return invert *= -1.0;
    }

    public double ConfigureInputs(boolean active, int choose){

        double marcha;
        double invert = DriverStation.getAlliance().get() == Alliance.Red ? -1.0 : 1.0;
        double gatilho = 0.7 + (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());

        if(active == true){ 

            marcha = gatilho;

            if(marcha <= 0){
                marcha *= -1.0;
            }

            switch (choose) {
                case 1:
                    
                    return controller.getLeftY() * invert * marcha;
            
                case 2:
                
                    return controller.getLeftX() * invert  * marcha;

                case 3:
                
                    return controller.getRightX() * marcha;
            }
        }
        return choose;
    }


    @Override
    public Trigger a(){
        return controller.a();
    }

    @Override
    public Trigger b(){
        return controller.b();
    }

    @Override
    public Trigger x(){
        return controller.x();
    }

    @Override
    public Trigger y(){
        return controller.y();
    }

    @Override
    public Trigger rightBumper(){
        return controller.rightBumper();
    }

    @Override
    public Trigger leftBumper(){
        return controller.leftBumper();
    }

    @Override
    public double automaticInverted(double value){
        Alliance invert = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;

        if(invert == Alliance.Red){
            return -value;
        }

        return value;
    }
    
    @Override
    public boolean TurboMode(){
        return controller.rightBumper().getAsBoolean();
    }

    @Override
    public double getMarcha(){
        return controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    }

    @Override
    public Trigger alingRobotOnReef() {
        return controller.pov(0);
    }

    @Override
    public double getLeftX(){
        if(TurboMode()){
            return getPerformByAlliance(controller.getLeftX());
        } else if(slowMode()){
            return getPerformByAlliance(controller.getLeftX()) * 0.2;
        } else{
            return getPerformByAlliance(controller.getLeftX()) * 0.6;
        }
    }

    @Override
    public double getLeftY(){
        if(TurboMode()){
            return getPerformByAlliance(controller.getLeftY());
        } else if(slowMode()){
            return getPerformByAlliance(controller.getLeftY())* 0.2;
        } else{
            return getPerformByAlliance(controller.getLeftY()) * 0.6;
        }
    }

    @Override
    public double getRightX(){
        if(TurboMode()){
            return -controller.getRightX() ;
        } else if(slowMode()){
            return -controller.getRightX() * 0.2;
        } else{
            return -controller.getRightX()  * 0.6;
        }
    }

    @Override
    public double getRightY(){
        if(TurboMode()){
            return -controller.getRightY();
        } else if(slowMode()){
            return -controller.getRightY()* 0.2;
        } else{
            return -controller.getRightY() * 0.6;
        }
    }

    @Override
    public boolean slowMode() {
        return controller.leftBumper().getAsBoolean();
    }

    @Override
    public Trigger emergencyInvert(){
        return controller.start();
    }

    @Override
    public double getPerformByAlliance(double value) {
        var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;

        if(alliance == Alliance.Blue){
            return value *= -1;
        } else {
            return value;
        }
    }

    @Override
    public boolean joystickIsNothingUsingDrive() {
        return Util.inRange(getLeftY(), -Controllers.DEADBAND, Controllers.DEADBAND)
        && Util.inRange(getLeftX(), -Controllers.DEADBAND, Controllers.DEADBAND);
    }

    @Override
    public Trigger resetPigeon() {
        return controller.button(8);
    }
}