package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.FRC9485.Autonomous.autonomousChooser.AutoChooser;
import frc.FRC9485.controllers.DriverController;
import frc.FRC9485.controllers.MechanismJoystick;
import frc.FRC9485.controllers.MechanismKeyBoard;
import frc.FRC9485.utils.RegisterNamedCommands;
import frc.FRC9485.utils.JoystickChooser.JoystickChooser;
import frc.robot.Constants.Components;
import frc.robot.commands.level.intake.SetIntakeSpeed;
import frc.robot.commands.swerveUtils.AlingToTarget;
import frc.robot.commands.swerveUtils.ResetPigeon;
import frc.robot.commands.swerveUtils.TurnRobot;
import frc.robot.subsystems.Mechanism.SuperStructure;
import frc.robot.subsystems.Mechanism.SuperStructure.StatesToScore;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {

  private final DriverController driverController;
  private final MechanismJoystick mechanismController;
  private final MechanismKeyBoard mechanismKeyboard;

  private final SwerveSubsystem swerve;
  private final IntakeSubsystem intakeSubsystem;
  
  private final SuperStructure superStructure;

  private final JoystickChooser joystickChooser;
  private final AutoChooser autoChooser;
  private final String mechanismSelected;

  private final RegisterNamedCommands registerNamedCommands;

  public RobotContainer() {
    this.driverController = DriverController.getInstance();
    this.mechanismController = MechanismJoystick.getInstance();
    this.mechanismKeyboard = MechanismKeyBoard.getInstance();

    this.swerve = SwerveSubsystem.getInstance();
    this.intakeSubsystem = IntakeSubsystem.getInstance();
    this.registerNamedCommands = RegisterNamedCommands.getInstance();
    this.configureNamedCommands();

    this.superStructure = SuperStructure.getInstance();

    this.joystickChooser = new JoystickChooser();
    this.autoChooser = AutoChooser.getInstance();
    this.mechanismSelected = joystickChooser.getSelected();

    swerve.setDefaultCommand(swerve.driveRobot(
      () -> driverController.getLeftY(),
      () -> driverController.getLeftX(),
      () -> driverController.getRightX(),
      frc.robot.Constants.swerve.FIELD_ORIENTED
    ));

    if(mechanismSelected == "joystick"){
      configureJoystickMechanismBindings();
      intakeSubsystem.setDefaultCommand(intakeSubsystem.throwCoral());
    } else {
      configureKeyBoardMechanismBindings();
    }

    configureDriveBindings();
  }

  private void configureNamedCommands(){
    registerNamedCommands.configureNamedCommands();
  }

  private void configureDriveBindings() {
    
    driverController.alingRobotOnReef().whileTrue(new AlingToTarget());

    driverController.resetPigeon().onTrue(new ResetPigeon()); 

    driverController.emergencyInvert().onTrue(new InstantCommand(() ->{
        driverController.Invert();
      }));
  }

  private void configureKeyBoardMechanismBindings(){

    mechanismKeyboard.L1Button().onTrue(new ParallelCommandGroup(
      superStructure.scorePieceOnLevel(StatesToScore.L1),
      new SetIntakeSpeed()
    ));
    mechanismKeyboard.L2Button().onTrue(superStructure.scorePieceOnLevel(StatesToScore.L2));    
    mechanismKeyboard.L3Button().onTrue(superStructure.scorePieceOnLevel(StatesToScore.L3));
    mechanismKeyboard.L4Button().onTrue(superStructure.scorePieceOnLevel(StatesToScore.L4));
    
    mechanismKeyboard.algae_L2().onTrue(superStructure.scorePieceOnLevel(StatesToScore.ALGAE_L2));
    mechanismKeyboard.algae_L3().onTrue(superStructure.scorePieceOnLevel(StatesToScore.ALGAE_L3));
    mechanismKeyboard.Processador().onTrue(superStructure.scorePieceOnLevel(StatesToScore.PROCESSOR));

    mechanismKeyboard.throwCoral().whileTrue(new SetIntakeSpeed(0.8));
    mechanismKeyboard.getAlgae().whileTrue(new SetIntakeSpeed(-0.1));
  }
  
  private void configureJoystickMechanismBindings(){ 
    
    mechanismController.L1Button().onTrue(new ParallelCommandGroup(
      superStructure.scorePieceOnLevel(StatesToScore.L1),
      new SetIntakeSpeed()
    ));
    mechanismController.L2Button().onTrue(superStructure.scorePieceOnLevel(StatesToScore.L2));
    mechanismController.L3Button().onTrue(superStructure.scorePieceOnLevel(StatesToScore.L3));
    mechanismController.L4Button().onTrue(superStructure.scorePieceOnLevel(StatesToScore.L4));    

    mechanismController.L2Algae().onTrue(superStructure.scorePieceOnLevel(StatesToScore.ALGAE_L2));
    mechanismController.L3Algae().onTrue(superStructure.scorePieceOnLevel(StatesToScore.ALGAE_L3));
    mechanismController.ProcessorButton().onTrue(superStructure.scorePieceOnLevel(StatesToScore.PROCESSOR));
  }

  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand(Components.AUTO, true);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
