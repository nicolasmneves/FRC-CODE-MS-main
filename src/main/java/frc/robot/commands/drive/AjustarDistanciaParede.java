package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Odometry.lidar.LidarLiteV3;
import frc.robot.subsystems.Drive.swerve.SwerveSubsystem;

// ============ AJUSTAR DISTANCIA DA PAREDE PELO LIDAR ============
// Anda para frente/tras ate o LIDAR medir a distancia pedida e para.
// Sem leitura do LIDAR o robo fica parado e o comando termina.
public class AjustarDistanciaParede extends Command {

    private static final double KP = 2.0;              // (m/s) por metro de erro
    private static final double VELOCIDADE_MAX = 1.0;  // m/s
    private static final double TOLERANCIA_M = 0.02;   // 2 cm
    private static final int CICLOS_ESTAVEIS = 5;

    private final SwerveSubsystem drive;
    private final LidarLiteV3 lidar;
    private final double alvoM;
    private int estaveis;

    /** @param distanciaMm distancia desejada da parede, em mm */
    public AjustarDistanciaParede(SwerveSubsystem drive, LidarLiteV3 lidar, double distanciaMm) {
        this.drive = drive;
        this.lidar = lidar;
        this.alvoM = distanciaMm / 1000.0;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        estaveis = 0;
    }

    @Override
    public void execute() {
        if (!lidar.temLeitura()) {
            drive.drive(new ChassisSpeeds());
            return;
        }
        double erro = lidar.getDistanciaM() - alvoM; // + = longe da parede, anda para frente
        double vx = MathUtil.clamp(KP * erro, -VELOCIDADE_MAX, VELOCIDADE_MAX);
        drive.drive(new ChassisSpeeds(vx, 0, 0));
        estaveis = Math.abs(erro) < TOLERANCIA_M ? estaveis + 1 : 0;
    }

    @Override
    public boolean isFinished() {
        return !lidar.temLeitura() || estaveis >= CICLOS_ESTAVEIS;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds());
    }
}
