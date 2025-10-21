package frc.FRC9485.utils.logger;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomBooleanLog extends BooleanLogEntry{

    private static boolean isFms;
    private String name;
    private Boolean loggedValue;
    
    public CustomBooleanLog(String name){
        super(DataLogManager.getLog(), name);
        this.name = name;
        CustomBooleanLog.isFms = DriverStation.isFMSAttached();
        this.loggedValue = true;
        append(false);
    }

    @Override
    public void append(boolean value) {
        if(DriverStation.isEnabled() && value != loggedValue){
            loggedValue = value;
            super.append(value);
        }
        if(!isFms){
            SmartDashboard.putBoolean(name, value);
        }
    }
}
