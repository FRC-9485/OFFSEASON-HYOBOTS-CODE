package frc.FRC9485.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IDDriverController {
    
    Trigger a();

    Trigger b();
    
    Trigger x();
    
    Trigger pov();

    Trigger rightBumper();

    Trigger leftBumper();

    double automaticInverted(double value);

    boolean TurboMode();

    boolean slowMode();

    Trigger alingRobotOnReef();

    double getLeftY();

    double getLeftX();

    double getRightY();

    double getRightX();

    double getMarcha();

    double Invert();

    Trigger emergencyInvert();

    double getPerformByAlliance(double value);

    boolean joystickIsNothingUsingDrive();

    Trigger resetPigeon();
}
