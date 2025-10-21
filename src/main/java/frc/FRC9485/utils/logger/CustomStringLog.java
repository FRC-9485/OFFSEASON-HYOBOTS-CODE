package frc.FRC9485.utils.logger;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomStringLog extends StringLogEntry{
    
    private static boolean isFms;
    private String loggedValue;
    private String name;

    public CustomStringLog(String name){
        super(DataLogManager.getLog(), name);
        this.name = name;
        CustomStringLog.isFms = DriverStation.getMatchNumber() > 0;
        this.loggedValue = "";
        this.append("");
    }

    @Override
    public void append(String value){
        if(DriverStation.isEnabled() && loggedValue != value){
            loggedValue = value;
            super.append(value);
        }
        if(!CustomStringLog.isFms){
            SmartDashboard.putString(this.name, value);
        }
    }
}
