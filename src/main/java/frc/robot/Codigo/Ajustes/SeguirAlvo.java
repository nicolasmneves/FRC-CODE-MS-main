package frc.robot.Codigo.Ajustes;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Codigo.Visao.CvCamera;
import frc.robot.subsystems.drive.swervedrive.SwerveSubsystem;

// ============ ALINHAR E SEGUIR O ALVO MARCADO (FILTRO DE KALMAN) ============
// Mantem a frente do robo no alvo marcado (cruz verde) enquanto o piloto anda.
// Trocar o alvo (RB) troca quem ele segue. Malha de controle PROPRIA (nao usa a AjusteMira).
//  VENDO    : cada frame corrige o filtro de Kalman (rumo + velocidade do alvo).
//  PREVENDO : sem frame, segue o rumo PREVISTO pela velocidade estimada.
//  BUSCANDO : sumiu por mais de TEMPO_PREVISAO, gira para o lado onde o alvo foi (0 = desliga).
public class SeguirAlvo extends Command {

    // ============ MALHA DE CONTROLE ============
    public static final double KP = 2.0;           // giro por radiano de erro
    public static final double KD = 0.0;           // amortecimento pela velocidade de giro real
    public static final double KV_ALVO = 0.4;      // antecipa a velocidade do alvo (0..1)
    public static final double GIRO_MAXIMO = 3.0;  // rad/s

    // ============ PERDA DO ALVO ============
    public static final double TEMPO_PREVISAO = 1.0; // s seguindo a previsao sem ver o alvo
    public static final double GIRO_BUSCA = 1.0;     // rad/s procurando depois disso (0 = nao procura)

    // ============ FILTRO DE KALMAN ============
    public static final double RUIDO_ACELERACAO = 6;     // maior = reage mais rapido (testado: 1)
    public static final double DESVIO_MEDIDA_GRAUS = 1;  // ruido da camera

    // ============ CAMERA ============
    public static final double SINAL_CAMERA = +1.0;     // +1 = camera desta montagem (invertida)
    public static final double ATRASO_EXTRA_MS = 0;    // atraso interno da camera
    public static final double CAMERA_LATERAL_M = 0.0;  // + = camera a esquerda do centro do robo
    public static final double LARGURA_OBJETO_M = 0.25; // largura real do objeto

    private final SwerveSubsystem drive;
    private final CvCamera cam;
    private final Supplier<ChassisSpeeds> conducao;
    private final FiltroKalmanRumo filtro = new FiltroKalmanRumo(RUIDO_ACELERACAO, DESVIO_MEDIDA_GRAUS);
    private final TimeInterpolatableBuffer<Rotation2d> historico = TimeInterpolatableBuffer.createBuffer(1.5);

    private double ultimoFrame;
    private double tUltimaMedida;
    private double tAnterior;
    private double ladoBusca;

    public SeguirAlvo(SwerveSubsystem drive, CvCamera cam, Supplier<ChassisSpeeds> conducao) {
        this.drive = drive;
        this.cam = cam;
        this.conducao = conducao;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        filtro.reset();
        ultimoFrame = -1;
        tUltimaMedida = -10;
        tAnterior = Timer.getFPGATimestamp();
        ladoBusca = 1;
    }

    @Override
    public void execute() {
        double agora = Timer.getFPGATimestamp();
        Rotation2d orientacao = drive.getHeading();
        historico.addSample(agora, orientacao);

        filtro.prever(agora - tAnterior);
        tAnterior = agora;
        medir(agora);

        double omega;
        String estado;
        double semVer = agora - tUltimaMedida;
        if (!filtro.isIniciado()) {
            omega = 0;
            estado = "sem alvo";
        } else if (semVer < TEMPO_PREVISAO) {
            omega = girarPara(filtro.getRumo(), filtro.getVelocidade(), orientacao);
            estado = semVer < 0.3 ? "vendo" : "prevendo";
        } else {
            omega = ladoBusca * GIRO_BUSCA;
            estado = "buscando";
        }

        ChassisSpeeds s = conducao.get();
        drive.driveFieldOriented(new ChassisSpeeds(s.vxMetersPerSecond, s.vyMetersPerSecond, omega));

        SmartDashboard.putString("Ajustes/Seguir/Estado", estado);
    }

    /** Frame novo da camera -> corrige o filtro com o rumo do alvo (no instante da foto). */
    private void medir(double agora) {
        CvCamera.Resultado r = cam.getResultado();
        if (r.alvo == null || r.timestampCaptura == ultimoFrame)
            return;
        ultimoFrame = r.timestampCaptura;
        double tFoto = r.timestampCaptura - ATRASO_EXTRA_MS / 1000.0;
        Rotation2d orientacaoFoto = historico.getSample(tFoto).orElse(drive.getHeading());
        double rumoFoto = orientacaoFoto.getRadians() + anguloObjeto(r);
        // leva a medida do instante da foto para agora, com a velocidade estimada
        double rumoAgora = rumoFoto + filtro.getVelocidade() * (agora - tFoto);
        filtro.corrigir(MathUtil.angleModulus(rumoAgora));
        tUltimaMedida = agora;
        if (Math.abs(filtro.getVelocidade()) > 0.05)
            ladoBusca = Math.signum(filtro.getVelocidade());
    }

    /** Angulo (rad, + = esquerda) para o objeto ficar na linha central do robo. */
    private double anguloObjeto(CvCamera.Resultado r) {
        double focal = cam.getFocalPixels() * r.larguraFrame / cam.getLarguraBaseCalibracao();
        double angulo = SINAL_CAMERA * Math.atan2(r.alvo.xCentro, focal);
        if (CAMERA_LATERAL_M != 0 && LARGURA_OBJETO_M > 0)
            angulo += Math.atan((CAMERA_LATERAL_M / LARGURA_OBJETO_M) * r.alvo.largura / focal);
        return angulo;
    }

    /** Giro (rad/s) para apontar para o rumo. */
    private double girarPara(double rumo, double velAlvo, Rotation2d orientacao) {
        double erro = MathUtil.angleModulus(rumo - orientacao.getRadians());
        double giroReal = drive.getFieldVelocity().omegaRadiansPerSecond;
        double desejado = KV_ALVO * velAlvo;
        double out = KP * erro + desejado + KD * (desejado - giroReal);
        return MathUtil.clamp(out, -GIRO_MAXIMO, GIRO_MAXIMO);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds());
    }
}
