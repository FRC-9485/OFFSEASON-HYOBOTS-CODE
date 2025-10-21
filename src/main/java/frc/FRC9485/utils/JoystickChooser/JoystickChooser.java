package frc.FRC9485.utils.JoystickChooser;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class JoystickChooser {
    
    LoggedDashboardChooser<String> joystickChooser;

    public JoystickChooser(){

        joystickChooser = new LoggedDashboardChooser<>("joystick chooser");

        joystickChooser.addDefaultOption("", "");
        joystickChooser.addDefaultOption("keyboard", "keyboard");
        joystickChooser.addOption("joystick", "joystick");
    }

    public String getSelected(){
        return joystickChooser.get();
    }
}
