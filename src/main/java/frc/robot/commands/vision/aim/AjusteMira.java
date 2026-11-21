package frc.robot.commands.vision.aim;

import frc.robot.subsystems.Odometry.vision.CvCamera;

// ============ AJUSTES DA MIRA PELA CAMERA ============
// Mantem a frente do robo olhando para o objeto (so direcao, sem distancia).
// Valores da mira usados pelo MiraControlador (botao Y). Ajuste aqui.
public final class AjusteMira {

    // ============ GANHOS ============
    // KP: giro por radiano de erro (rad/s por rad). Mais = acompanha mais rapido.
    // KD: amortecimento pela velocidade de giro real. Mais = menos oscilacao.
    // Tunado no robo (2026-10-23, giro no lugar, degraus de 10/20 graus):
    // KP 7 / KD 0.35 -> sobe em ~0.22-0.28 s, passa 2-4 graus, acomoda em ~0.2-0.5 s.
    // KP acima de ~9 so aumenta o overshoot (o swerve tem ~0.1 s de tempo morto para girar).
    public static final double KP = 10.0;
    public static final double KD = 0.35;

    // ============ LIMITES ============
    public static final double GIRO_MAXIMO = 2 * Math.PI;   // rad/s
    public static final double PERDA_ALVO = 0.5;            // s sem frame -> para de mirar
    // Atraso interno da camera (exposicao/USB) que o cscore nao mede. Somado ao atraso
    // medido para achar a orientacao do robo no instante real da foto.
    public static final double ATRASO_EXTRA_MS = 40;   // medido no robo (varredura 0/40/80/120)

    // ============ SENTIDO DA CAMERA ============
    // +1 = camera desta montagem (de cabeca para baixo, ja testado no robo). -1 = camera normal.
    public static final double SINAL_CAMERA = +1.0;

    // ============ CAMERA FORA DO CENTRO DO ROBO ============
    // Para o objeto entrar no CENTRO do robo mesmo com a camera deslocada para o lado.
    // CAMERA_LATERAL_M: distancia de lado da camera ate o centro do robo (m).
    //                   + = camera a ESQUERDA do centro | - = a DIREITA | 0 = camera no centro.
    // LARGURA_OBJETO_M: largura real do objeto (m), ex: base do cone.
    // Nao precisa de distancia: (deslocamento / largura do objeto) e igual perto ou longe.
    public static final double CAMERA_LATERAL_M = 0.0;
    public static final double LARGURA_OBJETO_M = 0.25;

    private AjusteMira() {
    }

    /**
     * Quanto o robo precisa girar (rad, + = esquerda) para o objeto ficar na linha
     * CENTRAL do robo (nao so no centro da camera).
     */
    static double anguloObjeto(CvCamera.Resultado r, CvCamera cam) {
        double focal = cam.getFocalPixels() * r.larguraFrame / cam.getLarguraBaseCalibracao();
        double anguloNaCamera = SINAL_CAMERA * Math.atan2(r.alvo.xCentro, focal);

        // linha central do robo vista pela camera: desvio = (camLateral / larguraObjeto) larguras do objeto
        if (CAMERA_LATERAL_M == 0 || LARGURA_OBJETO_M <= 0)
            return anguloNaCamera;
        double desvioCentro = Math.atan((CAMERA_LATERAL_M / LARGURA_OBJETO_M) * r.alvo.largura / focal);
        return anguloNaCamera + desvioCentro;
    }
}
