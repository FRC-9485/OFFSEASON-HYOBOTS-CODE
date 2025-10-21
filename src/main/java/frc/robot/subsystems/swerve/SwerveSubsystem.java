package frc.robot.subsystems.swerve;

import java.io.File;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC9485.utils.logger.CustomBooleanLog;
import frc.FRC9485.utils.logger.CustomDoubleLog;
import frc.robot.Constants.swerve;
import frc.robot.subsystems.vision.LimelightConfig;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import edu.wpi.first.math.trajectory.Trajectory;

public class SwerveSubsystem extends SubsystemBase implements SwerveIO{

  private SwerveDrive swerveDrive;
  private Pigeon2 pigeon;
  private PIDController xPID;
  private PIDController yPID;
  private ProfiledPIDController profilePid;
  private HolonomicDriveController driveController;
  private LimelightConfig limelightConfig;
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private SwerveModuleState[] state;
  private SwerveModule[] module;

  private double direcaoX;
  private double direcaoY;
  private double rotacao;
  private double lastMovingTime;
  private boolean isMoving;
  private IdleMode currentIdleMode;

  private CustomBooleanLog swerveIsMoving;
  private CustomDoubleLog entradaX;
  private CustomDoubleLog entradaY;
  private CustomDoubleLog entradaRot;

  public static SwerveSubsystem mInstance = null;

  public record SwerveState(double XInput, double YInput, double rotation, boolean isMoving) {}
  private SwerveState swerveState;

  private SwerveSubsystem(File directory){

    try {
      this.pigeon = new Pigeon2(9);
    } catch (Exception e) {
      e.printStackTrace();
      DriverStation.reportError("Falha ao inicializar Pigeon2: " + e.getMessage(), false);
      this.pigeon = null;
    }

    try {
      this.swerveDrive = new SwerveParser(directory).createSwerveDrive(swerve.MAX_SPEED);

      if (this.swerveDrive != null && this.pigeon != null) {
        this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            this.swerveDrive.kinematics,
            this.pigeon.getRotation2d(),
            this.swerveDrive.getModulePositions(),
            new Pose2d());

      } else {
        DriverStation.reportWarning("swerveDrive ou pigeon não inicializado — poseEstimator não criado", false);
        this.swerveDrivePoseEstimator = null;
      }
    } catch (Exception e) {
      e.printStackTrace();
      DriverStation.reportError("Erro criando SwerveDrive: " + e.getMessage(), false);
      this.swerveDrive = null;
      this.swerveDrivePoseEstimator = null;
    }

    xPID = new PIDController(0.1, 0, 0);
    yPID = new PIDController(0.1, 0, 0);
    profilePid = new ProfiledPIDController(0.1, 0.0, 0,
        new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    xPID.setTolerance(0.05);
    yPID.setTolerance(0.05);
    profilePid.setTolerance(0.05);
    driveController = new HolonomicDriveController(xPID, yPID, profilePid);
    driveController.setEnabled(true);

    this.setupPathPlanner();

    this.direcaoX = 0;
    this.direcaoY = 0;
    this.rotacao = 0;
    this.lastMovingTime = 0;
    this.isMoving = false;
    this.currentIdleMode = IdleMode.kBrake;

    this.swerveIsMoving = new CustomBooleanLog("swerve/ isMoving");
    this.entradaX = new CustomDoubleLog("swerve/X");
    this.entradaY = new CustomDoubleLog("swerve/Y");
    this.entradaRot = new CustomDoubleLog("swerve/rotation");

    this.swerveState = new SwerveState(direcaoX, direcaoY, rotacao, isMoving);
  }

  public static SwerveSubsystem getInstance(){
    if(mInstance == null){
      mInstance = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    }
    return mInstance;
  }
  
  @Override
  public SwerveState getState(){
    return swerveState;
  }

  @Override
  public void periodic() {
    if (swerveDrive != null) {
      try {
        swerveDrive.updateOdometry();
      } catch (Exception e) {
        e.printStackTrace();
        DriverStation.reportError("Erro em updateOdometry(): " + e.getMessage(), false);
      }
    } else {
      DriverStation.reportWarning("swerveDrive == null em periodic()", false);
    }
  
    if (swerveDrivePoseEstimator != null && pigeon != null && swerveDrive != null) {
      try {
        swerveDrivePoseEstimator.update(pigeon.getRotation2d(), swerveDrive.getModulePositions());
        if (limelightConfig != null && limelightConfig.getHasTarget()) {
          Pose2d poseEstimated = limelightConfig.getEstimatedGlobalPose();
          swerveDrivePoseEstimator.addVisionMeasurement(poseEstimated, Timer.getFPGATimestamp());
        }
      } catch (Exception e) {
        e.printStackTrace();
        DriverStation.reportError("Erro ao atualizar poseEstimator: " + e.getMessage(), false);
      }
    }
  
    this.automaticSwerveMode();
  }
  
  @Override
  public boolean swerveIsMoving(){
    boolean isMoving = Math.abs(direcaoX) > 0.05 || Math.abs(direcaoY) > 0.05 || Math.abs(rotacao) > 0.05;
    this.isMoving = isMoving;

    return isMoving;
  }

  @Override
  public void automaticSwerveMode(){
    boolean moving = swerveIsMoving();

    if (moving) {
      swerveIsMoving.append(true);
      this.lastMovingTime = Timer.getFPGATimestamp();
      if (this.currentIdleMode != IdleMode.kCoast) {
        if (swerveDrive != null) swerveDrive.setMotorIdleMode(false); 
        this.currentIdleMode = IdleMode.kCoast;
      }
    } else {
      swerveIsMoving.append(false);
      if (Timer.getFPGATimestamp() - lastMovingTime > 1.0) {
        if (this.currentIdleMode != IdleMode.kBrake) {
          if (swerveDrive != null) swerveDrive.setMotorIdleMode(true);
          this.currentIdleMode = IdleMode.kBrake;
        }
      }
    }
  }

  @Override
  public SwerveDrive getSwerveDrive(){
    return swerveDrive;
  }

  @Override
  public SwerveDrivePoseEstimator getPoseEstimator(){
    return this.swerveDrivePoseEstimator;
  }

    public void setupPathPlanner(){
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
            
            boolean feedforwards = true;
            
            AutoBuilder.configure(
                this::getPose,
                this::resetOdometry, 
                this::getRobotRelativeSpeeds, 
                (speeds, feedforward) -> {
                    if (feedforwards) {
                    swerveDrive.drive(
                        speeds,
                        swerveDrive.kinematics.toSwerveModuleStates(speeds),
                        feedforward.linearForces()
                    );
                    } else
                    {
                    swerveDrive.setChassisSpeeds(speeds);
                    }}, 
                    new PPHolonomicDriveController(
                    new PIDConstants(0.15, 0.0, 0.0), 
                    new PIDConstants(0.6, 0.0, 0.0)),
                    config, 
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                },
                this 
                );
                } catch (Exception e) {
                e.printStackTrace();
                }
        }


  @Override
  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  @Override
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Comando de direção com controle de malha fechada ativado por padrão
   * @param X Fornecedor de velocidade no eixo X
   * @param Y Fornecedor de velocidade no eixo Y
   * @param rotation Fornecedor de velocidade de rotacao
   * @return Comando para dirigir o robô
   */
  public Command alternDriveCommand(DoubleSupplier X, DoubleSupplier Y, DoubleSupplier rotation) {
    return alternDriveCommand(X, Y, rotation, true); // Usa malha fechada por padrão
  }
  
  /**
   * Comando de direção com opção de escolher entre malha aberta ou fechada
   * @param X Fornecedor de velocidade no eixo X
   * @param Y Fornecedor de velocidade no eixo Y
   * @param rotation Fornecedor de velocidade de rotacao
   * @param useClosedLoop Se true, usa controle de malha fechada; se false, usa malha aberta
   * @return Comando para dirigir o robô
   */
  @Override
  public Command alternDriveCommand(DoubleSupplier X, DoubleSupplier Y, DoubleSupplier rotation, boolean useClosedLoop) {
    // return run(() -> {
    //   double xController = Math.pow(xLimiter.calculate(X.getAsDouble()), 3);
    //   double yController = Math.pow(yLimiter.calculate(Y.getAsDouble()), 3);
    //   double rotationValue = rotationLimiter.calculate(rotation.getAsDouble());
    //   Rotation2d rotation2d = Rotation2d.fromDegrees(0.0);
    //   if (swerveDrivePoseEstimator != null) {
    //     rotation2d = swerveDrivePoseEstimator.getEstimatedPosition().getRotation();
    //   }
      
    //   ChassisSpeeds targetSpeeds = swerveDrive.swerveController.getTargetSpeeds(
    //       xController, 
    //       yController, 
    //       rotationValue, 
    //       getGyroAccum().getRadians(), 
    //       swerve.MAX_SPEED);
      
    //       if (useClosedLoop) {
    //         Pose2d currentPose = getPose();
    //         double dt = 0.02;
          
    //         Pose2d desiredPose = new Pose2d(
    //             currentPose.getX() + targetSpeeds.vxMetersPerSecond * dt,
    //             currentPose.getY() + targetSpeeds.vyMetersPerSecond * dt,
    //             currentPose.getRotation().plus(new Rotation2d(targetSpeeds.omegaRadiansPerSecond * dt)));
          
    //         Trajectory.State desiredState = new Trajectory.State();
    //         desiredState.poseMeters = desiredPose;
    //         desiredState.velocityMetersPerSecond =
    //             Math.hypot(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond);
          
    //         Rotation2d desiredRotation = desiredPose.getRotation();
          
    //         ChassisSpeeds adjustedSpeeds = driveController.calculate(currentPose, desiredState, desiredRotation);
          
    //         driveFieldOriented(adjustedSpeeds);
    //       } else {
    //         driveFieldOriented(targetSpeeds);
    //       }
    // });
    return Commands.none();
  }

  /**
   * @param x eixo de x do joystick
   * @param y eixo de y do joystick
   * @param omega eixo de rotaçãp
   * @param fromField drive orientado ao campo
   * @return
  **/
  @Override
  public Command driveRobot(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, boolean fromField){
    return run(() ->{

      // this.direcaoX = xLimiter.calculate(x.getAsDouble()) * swerveDrive.getMaximumChassisVelocity();
      // this.direcaoY = yLimiter.calculate(y.getAsDouble()) * swerveDrive.getMaximumChassisVelocity();
      // this.rotacao = rotationLimiter.calculate(omega.getAsDouble()) * swerveDrive.getMaximumChassisAngularVelocity();

      
      this.direcaoX = x.getAsDouble() * swerveDrive.getMaximumChassisVelocity();
      this.direcaoY = y.getAsDouble() * swerveDrive.getMaximumChassisVelocity();
      this.rotacao = omega.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity();

      if(direcaoX != 0) entradaX.append(direcaoX);
        

      if(direcaoY != 0) entradaY.append(direcaoY);

      if(rotacao != 0) entradaRot.append(rotacao);

      double td = 0.02;
      ChassisSpeeds speed = fromField == true ? ChassisSpeeds.fromFieldRelativeSpeeds(direcaoX,
                                                                                      direcaoY,
                                                                                      rotacao,
                                                                                      pigeon.getRotation2d()) 
                                                                                      : new ChassisSpeeds(
                                                                                      direcaoX, 
                                                                                      direcaoY,
                                                                                      rotacao);

      ChassisSpeeds discretize = ChassisSpeeds.discretize(speed, td);
      state = swerveDrive.kinematics.toSwerveModuleStates(discretize);
      SwerveDriveKinematics.desaturateWheelSpeeds(state, swerve.MAX_SPEED);
      
      module = swerveDrive.getModules();
      for(int i = 0; i < state.length; i++){
        module[i].setDesiredState(state[i], true, true);
      }
      });
  }

  @Override
  public void driveFieldOriented(ChassisSpeeds speed){
    swerveDrive.driveFieldOriented(speed);
  }

  @Override
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(scope0To360(pigeon.getYaw().getValueAsDouble()));
  }

  @Override
  public Command getAutonomousCommand(String name, boolean altern){
    if(altern){
      return AutoBuilder.buildAuto(name);
    }
    return new PathPlannerAuto(name);
  }
  
  @Override
  public void stopSwerve(){
    this.driveFieldOriented(new ChassisSpeeds());
  }

  public Rotation2d getGyroAccum(){
    double acumulo = pigeon.getAccumGyroX().getValueAsDouble();

    if(acumulo > 360){
      pigeon.reset();
    }

    acumulo = Math.IEEEremainder(acumulo, 360);

    return Rotation2d.fromDegrees(acumulo);
  }

  @Override
  public double getYaw(){
    return pigeon.getYaw().getValueAsDouble();
  }

  @Override
  public void setMotorBrake(boolean brake){
    swerveDrive.setMotorIdleMode(brake);
  }

  @Override
  public void resetOdometry(Pose2d pose){
   swerveDrive.resetOdometry(pose);
  }

  @Override
  public void drive(Translation2d translation2d, double rotation, boolean fieldOriented){
    swerveDrive.drive(translation2d, rotation, fieldOriented, false);
  }

  @Override
  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }
  
  /**
   * Configura os ganhos PID para o controlador holonômico de malha fechada
   * @param xKp Ganho proporcional para o controlador X
   * @param xKi Ganho integral para o controlador X
   * @param xKd Ganho derivativo para o controlador X
   * @param yKp Ganho proporcional para o controlador Y
   * @param yKi Ganho integral para o controlador Y
   * @param yKd Ganho derivativo para o controlador Y
   * @param rotKp Ganho proporcional para o controlador de rotacao
   * @param rotKi Ganho integral para o controlador de rotacao
   * @param rotKd Ganho derivativo para o controlador de rotacao
   */
  public void configurePIDGains(double xKp, double xKi, double xKd,
                               double yKp, double yKi, double yKd,
                               double rotKp, double rotKi, double rotKd) {

    xPID.setP(xKp);
    xPID.setI(xKi);
    xPID.setD(xKd);
    
    yPID.setP(yKp);
    yPID.setI(yKi);
    yPID.setD(yKd);
    
    profilePid.setP(rotKp);
    profilePid.setI(rotKi);
    profilePid.setD(rotKd);
  }

  @Override
  public double scope0To360(double value){
    value %= 360;

    return value;
  }
  
  public boolean atReference() {
    return xPID.atSetpoint() && yPID.atSetpoint() && profilePid.atGoal();
  }
}