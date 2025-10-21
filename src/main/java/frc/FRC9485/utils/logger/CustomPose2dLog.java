package frc.FRC9485.utils.logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class CustomPose2dLog{
    
    private static boolean isFms;

    private Pose2d loggedValue;

    private StructPublisher<Pose2d> publisher;

    private StructLogEntry<Pose2d> logEntry;

    public CustomPose2dLog(String name){
        this.logEntry = StructLogEntry.create(DataLogManager.getLog(), name, Pose2d.struct);
        this.publisher = NetworkTableInstance.getDefault().getStructTopic(name, Pose2d.struct).publish();
        CustomPose2dLog.isFms = DriverStation.getMatchNumber() > 0;
        this.loggedValue = new Pose2d(new Translation2d(100, 100), new Rotation2d());
    }

    public void appendRadians(Pose2d pose2d){
        if(!pose2d.equals(loggedValue)){
            this.loggedValue = pose2d;
            this.logEntry.append(pose2d);
        }
        if(!CustomPose2dLog.isFms){
            publisher.set(pose2d);
        }
    }

    public void appendDeegrees(Pose2d pose2d){
        this.logEntry.append(pose2d);
        if(!CustomPose2dLog.isFms){
            publisher.set(pose2d);
        }
    }
}
