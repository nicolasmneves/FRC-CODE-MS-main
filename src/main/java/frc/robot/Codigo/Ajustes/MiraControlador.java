package frc.robot.Codigo.Ajustes;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Codigo.Visao.CvCamera;
import frc.robot.subsystems.drive.swervedrive.SwerveSubsystem;

// ============ CONTROLADOR DA MIRA (SO DIRECAO, SEM DISTANCIA) ============
// Mantem a frente do robo "olhando" para o objeto enquanto a pessoa pilota.
// Por ciclo de 20 ms:
//  1. Guarda a orientacao do robo num historico.
//  2. Frame novo: rumo do alvo = orientacao NO INSTANTE DA FOTO + angulo do objeto
//     na imagem (compensa o atraso da camera, sem precisar de distancia).
//  3. Gira pelo giroscopio (rapido, sem atraso):
//     giro = KP * (rumoAlvo - orientacaoAtual) - KD * giroReal
public class MiraControlador {

    private final SwerveSubsystem drive;
    private final CvCamera cam;
    private final TimeInterpolatableBuffer<Rotation2d> historico = TimeInterpolatableBuffer.createBuffer(1.5);

    private Rotation2d rumoAlvo = null;
    private double ultimoFrame = -1;
    private double tUltimoAlvo = -10;
    private boolean temAlvo = false;

    public MiraControlador(SwerveSubsystem drive, CvCamera cam) {
        this.drive = drive;
        this.cam = cam;
    }

    /** Chamar ao iniciar o comando. */
    public void iniciar() {
        rumoAlvo = null;
        ultimoFrame = -1;
        tUltimoAlvo = -10;
        temAlvo = false;
    }

    /** true se ha alvo valido (visto ha menos de PERDA_ALVO s). */
    public boolean temAlvo() {
        return temAlvo;
    }

    /** Velocidade de giro (rad/s, + = anti-horario) para olhar para o objeto. 0 sem alvo. */
    public double omega() {
        double agora = Timer.getFPGATimestamp();
        Rotation2d orientacao = drive.getHeading();
        historico.addSample(agora, orientacao);
        atualizarRumo(orientacao, agora);

        if (!temAlvo)
            return 0;

        double erro = MathUtil.angleModulus(rumoAlvo.getRadians() - orientacao.getRadians());
        double giroReal = drive.getFieldVelocity().omegaRadiansPerSecond;
        return MathUtil.clamp(AjusteMira.KP * erro - AjusteMira.KD * giroReal,
                -AjusteMira.GIRO_MAXIMO, AjusteMira.GIRO_MAXIMO);
    }

    /** Frame novo -> rumo absoluto do alvo, usando a orientacao do instante da foto. */
    private void atualizarRumo(Rotation2d orientacao, double agora) {
        CvCamera.Resultado r = cam.getResultado();
        boolean frameNovo = r.timestampCaptura != ultimoFrame;
        if (r.alvo != null && frameNovo && agora - r.timestampCaptura < AjusteMira.PERDA_ALVO) {
            ultimoFrame = r.timestampCaptura;
            double atrasoExtra = AjusteMira.ATRASO_EXTRA_MS / 1000.0;
            Rotation2d orientacaoFoto = historico.getSample(r.timestampCaptura - atrasoExtra).orElse(orientacao);
            rumoAlvo = orientacaoFoto.plus(new Rotation2d(AjusteMira.anguloObjeto(r, cam)));
            tUltimoAlvo = agora;
        }
        temAlvo = rumoAlvo != null && agora - tUltimoAlvo < AjusteMira.PERDA_ALVO;
    }
}
