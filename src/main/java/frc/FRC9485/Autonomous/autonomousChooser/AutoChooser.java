package frc.FRC9485.Autonomous.autonomousChooser;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoChooser implements AutoChooserIO{

    LoggedDashboardChooser<String> autoChooser;
    public static AutoChooser mInstance = null;

    private AutoChooser(){
        this.autoChooser = new LoggedDashboardChooser<>("sendable chooser");

        this.autoChooser.addDefaultOption("is not used", "null");
        this.autoChooser.addOption("center autonomous", "New Auto");
    }

    public static AutoChooser getInstance(){
        if(mInstance == null){
            mInstance = new AutoChooser();
        }
        return mInstance;
    }

    @Override
    public String getPathName() {
        return this.autoChooser.get();
    }

    @Override
    public void addAutoNameCommand(String name, String pathName) {
        this.autoChooser.addOption(name, pathName);
    }
}
