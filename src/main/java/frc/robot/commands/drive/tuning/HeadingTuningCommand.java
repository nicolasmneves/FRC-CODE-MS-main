package frc.robot.commands.drive.tuning;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.swerve.SwerveSubsystem;
import swervelib.SwerveController;
import frc.robot.subsystems.Drive.assist.LimitadorGiro;
import frc.robot.constants.Constants;

// ============ TUNING DO PID DE RUMO (ANALÓGICO DIREITO) ============
// Roda no modo Test quando /Tuning/Mode = 1 (ver RobotContainer.getTestCommand()).
// O robô fica parado no lugar e só gira até o rumo de /Tuning/Heading/AlvoGraus (graus, campo).
// kP e kD são aplicados AO VIVO no thetaController do YAGSL, o mesmo que o teleop usa no modo
// "apontar"; depois copie os valores bons para deploy/swerve/neo/controllerproperties.json.
// Unidade do YAGSL: velocidade angular = PID(erro em rad) * maxAngularVelocity (rad/s).
public class HeadingTuningCommand extends Command
{
    private final SwerveSubsystem drivebase;
    private final DoubleEntry alvoGraus, kP, kD, subida, descida;
    private final LimitadorGiro limitador;
    private final DoublePublisher atualPub, erroPub, omegaPub;

    public HeadingTuningCommand(SwerveSubsystem drivebase, LimitadorGiro limitador)
    {
        this.drivebase = drivebase;
        this.limitador = limitador;
        PIDController pid = drivebase.getSwerveDrive().getSwerveController().thetaController;
        NetworkTable t = NetworkTableInstance.getDefault().getTable("Tuning/Heading");
        alvoGraus = t.getDoubleTopic("AlvoGraus").getEntry(0);
        kP = t.getDoubleTopic("kP").getEntry(pid.getP());
        kD = t.getDoubleTopic("kD").getEntry(pid.getD());
        kP.setDefault(pid.getP());
        kD.setDefault(pid.getD());
        // limites de aceleração do giro (rad/s²), os mesmos do teleop (LimitadorGiro)
        subida = t.getDoubleTopic("GiroSubida").getEntry(Constants.DrivebaseConstants.GIRO_SUBIDA);
        descida = t.getDoubleTopic("GiroDescida").getEntry(Constants.DrivebaseConstants.GIRO_DESCIDA);
        subida.setDefault(Constants.DrivebaseConstants.GIRO_SUBIDA);
        descida.setDefault(Constants.DrivebaseConstants.GIRO_DESCIDA);
        atualPub = t.getDoubleTopic("AtualGraus").publish();
        erroPub = t.getDoubleTopic("ErroGraus").publish();
        omegaPub = t.getDoubleTopic("OmegaRadS").publish();
        addRequirements(drivebase);
    }

    @Override
    public void initialize()
    {
        // começa segurando o rumo atual: nada se mexe até alguém mudar o alvo
        alvoGraus.set(drivebase.getHeading().getDegrees());
    }

    @Override
    public void execute()
    {
        SwerveController ctrl = drivebase.getSwerveDrive().getSwerveController();
        ctrl.thetaController.setP(kP.get());
        ctrl.thetaController.setD(kD.get());

        double atual = drivebase.getHeading().getRadians();
        double alvo = Math.toRadians(alvoGraus.get());
        double max = ctrl.config.maxAngularVelocity;
        limitador.configurar(subida.get(), descida.get());
        double omega = limitador.calcular(MathUtil.clamp(ctrl.headingCalculate(atual, alvo), -max, max));
        drivebase.getSwerveDrive().drive(new ChassisSpeeds(0, 0, omega));

        atualPub.set(Math.toDegrees(atual));
        erroPub.set(MathUtil.inputModulus(alvoGraus.get() - Math.toDegrees(atual), -180, 180));
        omegaPub.set(omega);
    }

    @Override
    public void end(boolean interrupted)
    {
        drivebase.getSwerveDrive().drive(new ChassisSpeeds());
    }
}
