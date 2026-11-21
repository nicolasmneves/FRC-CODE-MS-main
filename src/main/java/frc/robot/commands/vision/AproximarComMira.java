package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.vision.aim.AjusteMira;
import frc.robot.commands.vision.aim.MiraControlador;
import frc.robot.subsystems.Odometry.vision.CvCamera;
import frc.robot.subsystems.Mechanisms.intake.SensorFestoSOEG;
import frc.robot.subsystems.Drive.swerve.SwerveSubsystem;

// ============ TRAVAR MIRA E CORRER ATE O ALVO ============
// Anda para FRENTE mirando no objeto ate ele entrar no intake. Tres fases:
//  1. CORRIDA : vendo o objeto. Rapido com ele pequeno, desacelerando conforme ele cresce
//               na imagem (altura da caixa / altura da imagem).
//  2. EXTRA   : quando o objeto enche a imagem (ALTURA_FINAL) ou some ja grande (entrou
//               embaixo do robo), anda mais DISTANCIA_EXTRA pela odometria, desacelerando.
//  3. SENSOR  : depois da distancia extra, segue devagar e reto ate o sensor Festo = true
//               (confirmacao). Seguranca: no maximo DISTANCIA_MAX_SENSOR nessa fase.
public class AproximarComMira extends Command {

    // ============ FASE 1: CORRIDA ============
    public static final double VELOCIDADE_MAXIMA = 3.0; // m/s com o objeto pequeno (longe)
    public static final double VELOCIDADE_MINIMA = 0.6; // m/s com o objeto grande (perto)
    public static final double ACELERACAO = 6.0; // m/s^2 na largada (evita patinar)
    public static final double ALTURA_FINAL = 0.95; // fracao da imagem: vai para a fase EXTRA
    public static final double ALTURA_ENTRANDO = 0.55; // acima disso, sumir = "entrou no robo"

    // ============ FASE 2: DISTANCIA EXTRA ============
    public static final double DISTANCIA_EXTRA = 0.40; // m a mais depois de chegar perto
    public static final double KP_FINAL = 3.0; // (m/s)/m: desacelera nos ultimos cm
    public static final double VELOCIDADE_FINAL_MIN = 0.19; // m/s, nao travar antes do fim

    // ============ FASE 3: ATE O SENSOR ============
    public static final double VELOCIDADE_SENSOR = 0.3; // m/s reto ate o sensor ficar true
    public static final double DISTANCIA_MAX_SENSOR = 0.5; // m, seguranca se o sensor nao disparar
    /** true = sensor true nas fases 1 ou 2 tambem para (objeto ja entrou, nao empurrar). */
    public static final boolean PARAR_SE_SENSOR_ANTES = true;

    public static final double TEMPO_MAXIMO = 5.0; // s, seguranca do comando inteiro

    private enum Fase { CORRIDA, EXTRA, SENSOR }

    private final SwerveSubsystem drive;
    private final CvCamera cam;
    private final SensorFestoSOEG sensor;
    private final MiraControlador mira;
    private final SlewRateLimiter rampa = new SlewRateLimiter(ACELERACAO, -100, 0);

    private Fase fase;
    private double inicio;
    private double ultimaAltura;
    private Translation2d inicioFase; // posicao ao entrar na fase EXTRA ou SENSOR
    private boolean terminou;

    public AproximarComMira(SwerveSubsystem drive, CvCamera cam, SensorFestoSOEG sensor) {
        this.drive = drive;
        this.cam = cam;
        this.sensor = sensor;
        this.mira = new MiraControlador(drive, cam);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        mira.iniciar();
        rampa.reset(0);
        inicio = Timer.getFPGATimestamp();
        ultimaAltura = alturaObjeto();
        fase = Fase.CORRIDA;
        terminou = false;
        if (sensor.getSensor())
            terminar("objeto ja esta no intake");
        else if (ultimaAltura < 0)
            terminar("sem objeto ao apertar");
        else
            status("1/3 correndo ate o alvo");
    }

    @Override
    public void execute() {
        double omega = mira.omega();
        double altura = alturaObjeto();
        double vx = 0;

        switch (fase) {
        case CORRIDA: {
            double vMax = VELOCIDADE_MAXIMA, vMin = VELOCIDADE_MINIMA;
            double alturaFinal = ALTURA_FINAL, entrando = ALTURA_ENTRANDO;
            if (altura >= 0) {
                // rapido longe, mais devagar conforme o objeto cresce
                ultimaAltura = altura;
                double falta = MathUtil.clamp((alturaFinal - altura) / alturaFinal, 0, 1);
                vx = vMin + (vMax - vMin) * falta;
                if (altura >= alturaFinal)
                    irPara(Fase.EXTRA, "2/3 objeto encheu a imagem -> distancia extra");
            } else if (ultimaAltura >= entrando) {
                vx = vMin;
                irPara(Fase.EXTRA, "2/3 objeto entrou embaixo do robo -> distancia extra");
            } else {
                terminar("perdeu o objeto (altura " + Math.round(ultimaAltura * 100) + "%)");
            }
            break;
        }
        case EXTRA: {
            // mais DISTANCIA_EXTRA pela odometria, desacelerando
            double resta = DISTANCIA_EXTRA - andadoNaFase();
            vx = MathUtil.clamp(KP_FINAL * resta, VELOCIDADE_FINAL_MIN, VELOCIDADE_MINIMA);
            if (altura < 0)
                omega = 0; // sem objeto na imagem: so segue reto
            if (resta <= 0)
                irPara(Fase.SENSOR, "3/3 distancia extra feita -> ate o sensor");
            break;
        }
        case SENSOR: {
            // devagar e reto ate o sensor confirmar
            vx = VELOCIDADE_SENSOR;
            if (altura < 0)
                omega = 0;
            if (andadoNaFase() > DISTANCIA_MAX_SENSOR)
                terminar("sensor nao detectou em " + DISTANCIA_MAX_SENSOR + " m");
            break;
        }
        }

        // sensor do intake: confirma na fase 3 (ou para antes, se configurado)
        if (sensor.getSensor() && (fase == Fase.SENSOR || PARAR_SE_SENSOR_ANTES))
            terminar("objeto no intake (sensor) na fase " + fase);
        if (Timer.getFPGATimestamp() - inicio > TEMPO_MAXIMO)
            terminar("tempo maximo");

        vx = terminou ? 0 : rampa.calculate(vx);
        drive.drive(new ChassisSpeeds(vx, 0, terminou ? 0 : omega)); // referencial do robo
    }

    @Override
    public boolean isFinished() {
        return terminou;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds());
        if (interrupted && !terminou)
            status("cancelado");
    }

    private void irPara(Fase nova, String motivo) {
        fase = nova;
        inicioFase = drive.getPose().getTranslation();
        status(motivo);
    }

    /** Metros andados desde que a fase atual comecou. */
    private double andadoNaFase() {
        return inicioFase.getDistance(drive.getPose().getTranslation());
    }

    private void terminar(String motivo) {
        if (terminou)
            return; // guarda o primeiro motivo
        terminou = true;
        status(motivo);
    }

    /** Altura do objeto alvo / altura da imagem (0..1). -1 se nao ha objeto recente. */
    private double alturaObjeto() {
        CvCamera.Resultado r = cam.getResultado();
        boolean recente = Timer.getFPGATimestamp() - r.timestamp < AjusteMira.PERDA_ALVO;
        if (r.alvo == null || !recente || r.alturaFrame <= 0)
            return -1;
        return r.alvo.altura / (double) r.alturaFrame;
    }

    private void status(String s) {
        SmartDashboard.putString("Ajustes/Aproximar/Status", s);
        System.out.println("[Aproximar] " + s);
    }
}
