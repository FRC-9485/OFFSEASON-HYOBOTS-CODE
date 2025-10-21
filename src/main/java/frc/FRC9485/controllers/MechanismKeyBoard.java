package frc.FRC9485.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MechanismKeyBoard implements KeyBoardIO{
    
    private CommandXboxController controller;

    public static MechanismKeyBoard mInstance = null;

    private MechanismKeyBoard(){
        this.controller = new CommandXboxController(2);
    }

    public static MechanismKeyBoard getInstance(){
        if(mInstance == null){
            mInstance = new MechanismKeyBoard();
        }
        return mInstance;
    }

    @Override
    public Trigger L1Button() {
        return controller.button(1);
    }

    @Override
    public Trigger L2Button() {
        return controller.button(2);
    }

    @Override
    public Trigger L3Button() {
        return controller.button(3);
    }

    @Override
    public Trigger L4Button() {
        return controller.button(4);
    }

    @Override
    public Trigger algae_L2() {
        return controller.button(8);
    }

    @Override
    public Trigger algae_L3() {
        return controller.button(9);
    }

    @Override
    public Trigger Processador() {
        return controller.button(7);
    }

    @Override
    public Trigger throwCoral(){
        return controller.button(6);
    }

    @Override
    public Trigger getAlgae(){
        return controller.button(5);
    }
}
