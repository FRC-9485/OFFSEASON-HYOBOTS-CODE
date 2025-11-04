package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem.SwerveState;
import swervelib.SwerveDrive;

public interface SwerveIO {
    
    void drive(Translation2d translation, double rotation, boolean fieldOriented);

    Command driveRobot(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, boolean fieldOriented);

    void resetOdometry(Pose2d pose2d);

    void zeroGyro();

    void driveFieldOriented(ChassisSpeeds speeds);

    Command getAutonomousCommand(String path, boolean altern);

    double scope0To360(double value);

    void setMotorBrake(boolean brake);

    Command stopSwerve();

    Command alternDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, boolean clossedLoop);

    Pose2d getPose();

    void automaticSwerveMode();

    void setupPathPlanner();

    double getYaw();

    Rotation2d getHeading();

    boolean swerveIsMoving();

    ChassisSpeeds getRobotRelativeSpeeds();

    SwerveDrive getSwerveDrive();

    SwerveDrivePoseEstimator getPoseEstimator();

    SwerveState getState();

    boolean swerveIsStoped();

    void setState(SwerveModuleState[] state);

    Command putSwerveIn0();

    Pose2d getSidePose();
}
