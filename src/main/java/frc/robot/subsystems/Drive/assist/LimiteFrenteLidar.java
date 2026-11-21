package frc.robot.subsystems.Drive.assist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Odometry.lidar.LidarLiteV3;

// ============ LIMITE DA FRENTE PELO LIDAR (TORRE / MATRIZ) ============
// O LIDAR fica na frente do robô. Quando o robô está DE FRENTE para a estrutura do modo
// escolhido (Torre ou Matriz), ele não deixa a frente chegar mais perto que a distância
// configurada: o piloto pode empurrar o analógico à vontade que o robô desacelera e para
// na distância certa. Andar para trás e para os lados continua livre.
//
//   Torre            Robô            Matriz
//    ++               ^                ++
//
// O que ele verifica a cada ciclo:
//  1. ALINHADO   : o rumo do robô está dentro de TOLERANCIA_ANGULO do rumo da estrutura.
//  2. NA DISTÂNCIA: a leitura do LIDAR está dentro de TOLERANCIA_DISTANCIA da distância alvo.
//  3. PRONTO     : as duas coisas juntas (aparece no Elastic e pisca o LED).
//
// Só limita quando está ALINHADO: torto, o LIDAR pode estar medindo outra coisa.
//
// FÁCIL DE MUDAR: as distâncias e os rumos abaixo são os valores iniciais. Também dá para
// mudar AO VIVO no Elastic em "LimiteFrente/..." (vale até reiniciar o código; depois
// copie o valor bom para cá).
public class LimiteFrenteLidar {

    // ============ DISTÂNCIA DE CADA ESTRUTURA (cm, medida pelo LIDAR) ============
    // Leitura do LIDAR em que o robô deve parar. Se calibrar o offset do LIDAR para a
    // face do para-choque, isso vira a distância do para-choque até a estrutura.
    public static final double DISTANCIA_TORRE_CM = 30.0;
    public static final double DISTANCIA_MATRIZ_CM = 30.0;

    // ============ RUMO DE CADA ESTRUTURA (graus) ============
    // 0 = olhando para longe do piloto ("para frente"). Positivo = girado para a esquerda.
    // Na aliança vermelha o código soma 180° sozinho (o rumo 0° do vermelho é 180°).
    public static final double RUMO_TORRE_GRAUS = 0.0;
    public static final double RUMO_MATRIZ_GRAUS = 0.0;

    // ============ TOLERÂNCIAS ============
    public static final double TOLERANCIA_ANGULO_GRAUS = 5.0; // considera "de frente" até isso
    public static final double TOLERANCIA_DISTANCIA_CM = 3.0; // considera "na distância" até isso

    // ============ FREIO PERTO DA ESTRUTURA ============
    // Velocidade máxima para frente = KP_FREIO × (distância que falta). Ex.: faltando 50 cm,
    // até 1,5 m/s; faltando 10 cm, até 0,3 m/s; na distância, 0.
    public static final double KP_FREIO = 3.0; // (m/s) por metro que falta

    // ============ CHAVES DO ELASTIC ============
    private static final String K = "LimiteFrente/";
    private static final String K_DIST_TORRE = K + "DistanciaTorreCm";
    private static final String K_DIST_MATRIZ = K + "DistanciaMatrizCm";
    private static final String K_RUMO_TORRE = K + "RumoTorreGraus";
    private static final String K_RUMO_MATRIZ = K + "RumoMatrizGraus";
    private static final String K_ATIVO = K + "Ativo";

    private final LidarLiteV3 lidar;

    // Último estado calculado (para telemetria e LEDs).
    private boolean alinhado = false;
    private boolean naDistancia = false;
    private boolean limitando = false;
    private double erroAnguloGraus = 0;
    private double faltaCm = 0;

    public LimiteFrenteLidar(LidarLiteV3 lidar) {
        this.lidar = lidar;
        SmartDashboard.setDefaultNumber(K_DIST_TORRE, DISTANCIA_TORRE_CM);
        SmartDashboard.setDefaultNumber(K_DIST_MATRIZ, DISTANCIA_MATRIZ_CM);
        SmartDashboard.setDefaultNumber(K_RUMO_TORRE, RUMO_TORRE_GRAUS);
        SmartDashboard.setDefaultNumber(K_RUMO_MATRIZ, RUMO_MATRIZ_GRAUS);
        SmartDashboard.setDefaultBoolean(K_ATIVO, true);
    }

    // ============ VALORES ATUAIS (ELASTIC OU PADRÃO) ============

    /** Distância alvo (m) da estrutura do modo. */
    public double distanciaAlvoM(boolean modoTorre) {
        double cm = modoTorre
                ? SmartDashboard.getNumber(K_DIST_TORRE, DISTANCIA_TORRE_CM)
                : SmartDashboard.getNumber(K_DIST_MATRIZ, DISTANCIA_MATRIZ_CM);
        return cm / 100.0;
    }

    /** Rumo no CAMPO em que o robô fica de frente para a estrutura do modo (já com a aliança). */
    public Rotation2d rumoAlvo(boolean modoTorre) {
        double graus = modoTorre
                ? SmartDashboard.getNumber(K_RUMO_TORRE, RUMO_TORRE_GRAUS)
                : SmartDashboard.getNumber(K_RUMO_MATRIZ, RUMO_MATRIZ_GRAUS);
        var alianca = DriverStation.getAlliance();
        if (alianca.isPresent() && alianca.get() == DriverStation.Alliance.Red)
            graus += 180.0;
        return Rotation2d.fromDegrees(graus);
    }

    // ============ ATUALIZA O ESTADO (ALINHADO / NA DISTÂNCIA) ============
    private void avaliar(Rotation2d rumo, boolean modoTorre) {
        erroAnguloGraus = MathUtil.inputModulus(
                rumoAlvo(modoTorre).getDegrees() - rumo.getDegrees(), -180, 180);
        alinhado = Math.abs(erroAnguloGraus) <= TOLERANCIA_ANGULO_GRAUS;

        if (lidar.temLeitura()) {
            faltaCm = (lidar.getDistanciaM() - distanciaAlvoM(modoTorre)) * 100.0;
            naDistancia = Math.abs(faltaCm) <= TOLERANCIA_DISTANCIA_CM;
        } else {
            faltaCm = Double.NaN;
            naDistancia = false;
        }
    }

    // ============ LIMITA A VELOCIDADE PEDIDA PELO PILOTO ============
    /**
     * Recebe a velocidade pedida (referencial do CAMPO) e devolve a mesma velocidade com a
     * componente "para frente do robô" limitada perto da estrutura.
     */
    public ChassisSpeeds limitar(ChassisSpeeds campo, Rotation2d rumo, boolean modoTorre) {
        avaliar(rumo, modoTorre);
        limitando = false;

        if (!SmartDashboard.getBoolean(K_ATIVO, true) || !alinhado || !lidar.temLeitura())
            return campo;

        // velocidade do campo -> referencial do robô (x = frente)
        Translation2d robo = new Translation2d(campo.vxMetersPerSecond, campo.vyMetersPerSecond)
                .rotateBy(rumo.unaryMinus());

        double falta = lidar.getDistanciaM() - distanciaAlvoM(modoTorre); // m até a distância alvo
        double frenteMax = Math.max(0.0, KP_FREIO * falta);

        if (robo.getX() > frenteMax) {
            robo = new Translation2d(frenteMax, robo.getY());
            limitando = true;
        }

        // volta para o referencial do campo
        Translation2d volta = robo.rotateBy(rumo);
        return new ChassisSpeeds(volta.getX(), volta.getY(), campo.omegaRadiansPerSecond);
    }

    // ============ TELEMETRIA (chamar a cada ciclo) ============
    public void publicar(Rotation2d rumo, boolean modoTorre) {
        avaliar(rumo, modoTorre);
        SmartDashboard.putString(K + "Estrutura", modoTorre ? "TORRE" : "MATRIZ");
        SmartDashboard.putBoolean(K + "Alinhado", alinhado);
        SmartDashboard.putNumber(K + "ErroAnguloGraus", erroAnguloGraus);
        SmartDashboard.putBoolean(K + "NaDistancia", naDistancia);
        SmartDashboard.putNumber(K + "FaltaCm", faltaCm);
        SmartDashboard.putBoolean(K + "Limitando", limitando);
        SmartDashboard.putBoolean(K + "Pronto", isPronto());
    }

    // ============ CONSULTAS ============
    public boolean isAlinhado() {
        return alinhado;
    }

    public boolean isNaDistancia() {
        return naDistancia;
    }

    /** De frente para a estrutura e na distância certa. */
    public boolean isPronto() {
        return alinhado && naDistancia;
    }
}
