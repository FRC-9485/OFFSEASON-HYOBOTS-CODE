package frc.robot.commands.swerveUtils;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.LimelightConfig;
import frc.FRC9485.utils.Util;
import frc.robot.Constants.Components;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AlingToTarget extends Command {

    private final Pigeon2 pigeon;
    private final LimelightConfig limelight;
    private final SwerveSubsystem subsystem;
    private final PIDController xController;
    private final PIDController taController;
    private final PIDController rotationController;
    private double setpointX, setpointTA;

    private final Timer timer = new Timer();
    private boolean automaticSetpoint;

    // PID Constants - aumentadas para resposta mais rápida
    private static final double kP_ROTATION = 0.04;
    private static final double kI_ROTATION = 0.01;
    private static final double kD_ROTATION = 0.009;
    // private static final double kI_ROTATION = 0.01;
    // private static final double kD_ROTATION = 0.005;

    // private static final double kP_X = 0.52;
    // private static final double kI_X = 0.05;
    // private static final double kD_X = 0.00;

    private final double kP_X = 0.04;
    private final double kI_X = 0.01;
    private final double kD_X = 0.009;

    private final double kP_TA = 0.04;
    private final double kI_TA = 0.01;
    private final double kD_TA = 0.009;

    private final double ROTATION_SETPOINT = 0.01;
    private final double TOLERANCIA_ROTATION = 0.05;
    private final double TOLERANCIA_X = 0.05;
    private final double TOLERANCIA_TA = 1;

    private final double MAX_CORRECAO = 1;
    private final double MAX_CORRECAO_TA = 2;
    private final double ZONA_MORTA = 0.02;
    private final double TIMEOUT_SECONDS = 8.0;
    private final double TEMPO_MINIMO_ESTAVEL = 0.2;
    
    private int passo;
    private double angle;
    private double rotationOut;

    private double ultimaTx = 0.0;
    private double ultimoTempoMudanca = 0.0;

    public AlingToTarget(double setpointX, double setpointTA) {
        this(setpointX, setpointTA, true);
    }

    public AlingToTarget() {
        this(0, 0, false);
    }

    private AlingToTarget(double setpointX, double setpointTA, boolean automaticSetpoint) {
        this.limelight = LimelightConfig.getInstance();
        this.subsystem = SwerveSubsystem.getInstance();
        this.setpointX = setpointX;
        this.setpointTA = setpointTA;
        this.automaticSetpoint = automaticSetpoint;
        this.xController = new PIDController(kP_X, kI_X, kD_X);
        this.taController = new PIDController(kP_TA, kI_TA, kD_TA);
        this.rotationController = new PIDController(kP_ROTATION, kI_ROTATION, kD_ROTATION);
        this.pigeon = new Pigeon2(Components.PIGEON);

        SmartDashboard.putData("Limelight/PID_ROTATION", rotationController);
        SmartDashboard.putData("Limelight/PID_TA", taController);
        SmartDashboard.putData("Limelight/PID_TX", xController);

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        ultimaTx = 0.0;
        ultimoTempoMudanca = 0.0;

        if (automaticSetpoint) {
            double[] coords = limelight.getAprilTagCordenates();
            double tx = coords[0];
            double ta = 7.4;

            if (!Double.isNaN(tx) && !Double.isNaN(ta)) {
                this.setpointX = tx;
                this.setpointTA = ta;
            } else {
                System.out.println("Valores inválidos da Limelight, mantendo setpoints existentes.");
            }
        }

        passo = 1;

        rotationController.reset();
        xController.reset();
        taController.reset();

        xController.setSetpoint(setpointX);
        taController.setSetpoint(setpointTA);
        rotationController.setSetpoint(ROTATION_SETPOINT);

        rotationController.setTolerance(TOLERANCIA_ROTATION);
        xController.setTolerance(TOLERANCIA_X);
        taController.setTolerance(TOLERANCIA_TA);
    }

    @Override
    public void execute() {
        boolean hasTarget = limelight.getHasTarget();

        if (!hasTarget) {
            subsystem.drive(new Translation2d(), 0, true);
            return;
        }

        double tx = limelight.getTx();
        double ta = limelight.getTa();
        angle = pigeon.getYaw().getValueAsDouble();

        if (Double.isNaN(tx) || Double.isNaN(ta) || Double.isNaN(ta)) return;

        if (Math.abs(tx - ultimaTx) > 10.0) {
            ultimoTempoMudanca = timer.get();
        }
        ultimaTx = tx;

        if (Util.inRange(ta, 0.2, 0.6)) {
            ta *= 22;
        } else if (Util.inRange(ta, 0.61, 2.0)) {
            ta *= 12;
        }

        if (timer.get() - ultimoTempoMudanca > TEMPO_MINIMO_ESTAVEL) {
            double correcaoX = xController.calculate(tx, setpointX);
            double correcaoTA = taController.calculate(ta, setpointTA);
            rotationOut = rotationController.calculate(angle, 1);

            // Zona morta
            if (Math.abs(correcaoX) < ZONA_MORTA) correcaoX = 0;
            if (Math.abs(correcaoTA) < ZONA_MORTA) correcaoTA = 0;

            // Limite máximo de velocidade
            correcaoX = Math.min(Math.max(correcaoX, -MAX_CORRECAO), MAX_CORRECAO);
            correcaoTA = Math.min(Math.max(correcaoTA, -MAX_CORRECAO), MAX_CORRECAO);
            rotationOut = Math.min(Math.max(rotationOut, -MAX_CORRECAO_TA), MAX_CORRECAO_TA);

            if (angle >= 1.0 && angle <= -1.0) {
                passo = 2;
            }

            switch (passo) {
                case 1:
                    if (Util.inRange(angle, -2.5, 2.5)) { passo = 2; }
                    SmartDashboard.putNumber("angulo", angle);
                    SmartDashboard.putNumber("rotation", rotationOut);
                    subsystem.drive(new Translation2d(0,0), -rotationOut, true);
                    break;
            
                case 2:
                    if (Util.inRange(tx, -2, 2)) { passo = 3; }

                    Translation2d translation2d = new Translation2d(0, correcaoX);
                    subsystem.drive(translation2d, 0, true);
                    break;
                
                case 3:
                    if (Util.inRange(ta, 6.9, 7.4)) { passo = 4; }
                    translation2d = new Translation2d(-correcaoTA, 0);
                    subsystem.drive(translation2d, 0, true);
                    break;

                case 4:
                    if (Util.inRange(tx, -1, 1)) { passo = 5; }
                    translation2d = new Translation2d(0, correcaoX);
                    subsystem.drive(translation2d, 0, true);
                    break;

                default:
                    passo = 1;
                    break;
            }

            System.out.println("Passo atual: " + passo);

            // Translation2d translation2d = new Translation2d(correcaoTA, -correcaoX);
            // subsystem.drive(translation2d, -correcaoRot, true);
        }
    }

    @Override
    public boolean isFinished() {
        return (xController.atSetpoint() && taController.atSetpoint()) 
               || timer.hasElapsed(TIMEOUT_SECONDS);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.drive(new Translation2d(), 0, true);
        timer.stop();
        if (interrupted) {
            System.out.println("Alinhamento interrompido!");
        } else {
            System.out.println("Alinhamento concluído!");
        }
    }
}
