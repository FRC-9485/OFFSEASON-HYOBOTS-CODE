package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.Components;

public class LimelightConfig {
    
    private NetworkTable limelight;

    public static LimelightConfig mInstance = null;

    private LimelightConfig(String table){
        this.limelight = NetworkTableInstance.getDefault().getTable(table);
    }

    public static LimelightConfig getInstance(){
        if(mInstance ==  null){
            mInstance = new LimelightConfig(Components.LIMELIGHT);
        }
        return mInstance;
    }

    public boolean getHasTarget(){
        return limelight.getEntry("tv").getDouble(0)==1;
    }

    public double getTagId(){
        return limelight.getEntry("tid").getDouble(-1);
    }

    public double getTx(){
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public double getTy(){
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public double getTa(){
        return limelight.getEntry("ta").getDouble(0.0);
    }

    public boolean setLedMode(int mode){
        return limelight.getEntry("ledMode").setNumber(mode);
    }

    public double[] getRobotPose(){
        return limelight.getEntry("botPose").getDoubleArray(new double[6]);
    }

    public double[] getAprilTagCordenates(){
        return limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    }

    public Pose2d getEstimatedGlobalPose(){
       double[] blue = getBotPoseBlue();

       if(blue.length >= 6){
        return new Pose2d(
            blue[0],
            blue[1],
            Rotation2d.fromDegrees(blue[5])
        );
       }
       return new Pose2d();
    }

    public double[] getBotPoseBlue(){
        return limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }
}