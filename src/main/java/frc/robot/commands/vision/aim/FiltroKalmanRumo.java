package frc.robot.commands.vision.aim;

import edu.wpi.first.math.MathUtil;

// ============ FILTRO DE KALMAN DO RUMO DO ALVO ============
// Estima a DIRECAO do alvo no campo (rumo, rad) e a VELOCIDADE com que ela muda (rad/s),
// com modelo de velocidade constante:
//   prever(dt)   -> projeta o rumo para frente (usa quando nao ha frame novo)
//   corrigir(z)  -> junta a medida da camera, suavizando o ruido
// Sem frames, a previsao continua com a velocidade estimada: o robo nao "perde" o alvo.
public class FiltroKalmanRumo {

    /** Ruido do movimento do alvo: maior = acompanha mudancas rapidas, menor = mais suave. */
    private final double ruidoAceleracao;
    /** Ruido da medida da camera (rad^2). */
    private final double ruidoMedida;

    private double rumo;       // rad
    private double velocidade; // rad/s
    private double p00, p01, p10, p11; // covariancia
    private boolean iniciado = false;

    /**
     * @param ruidoAceleracao (rad/s^2)^2, ex: 1
     * @param desvioMedidaGraus desvio da medida da camera, ex: 2 graus
     */
    public FiltroKalmanRumo(double ruidoAceleracao, double desvioMedidaGraus) {
        this.ruidoAceleracao = ruidoAceleracao;
        double r = Math.toRadians(desvioMedidaGraus);
        this.ruidoMedida = r * r;
    }

    public void reset() {
        iniciado = false;
    }

    public boolean isIniciado() {
        return iniciado;
    }

    /** Projeta o estado dt segundos para frente. */
    public void prever(double dt) {
        if (!iniciado || dt <= 0)
            return;
        rumo = MathUtil.angleModulus(rumo + velocidade * dt);
        // P = F P F^T + Q (F = [[1, dt], [0, 1]])
        double n00 = p00 + dt * (p01 + p10) + dt * dt * p11;
        double n01 = p01 + dt * p11;
        double n10 = p10 + dt * p11;
        double q = ruidoAceleracao;
        p00 = n00 + q * dt * dt * dt / 3;
        p01 = n01 + q * dt * dt / 2;
        p10 = n10 + q * dt * dt / 2;
        p11 = p11 + q * dt;
    }

    /** Junta uma medida de rumo (rad) da camera. */
    public void corrigir(double rumoMedido) {
        if (!iniciado) {
            rumo = rumoMedido;
            velocidade = 0;
            p00 = ruidoMedida;
            p01 = p10 = 0;
            p11 = 1.0; // velocidade ainda desconhecida
            iniciado = true;
            return;
        }
        double inovacao = MathUtil.angleModulus(rumoMedido - rumo);
        double s = p00 + ruidoMedida;
        double k0 = p00 / s, k1 = p10 / s;
        rumo = MathUtil.angleModulus(rumo + k0 * inovacao);
        velocidade += k1 * inovacao;
        double n00 = (1 - k0) * p00, n01 = (1 - k0) * p01;
        double n10 = p10 - k1 * p00, n11 = p11 - k1 * p01;
        p00 = n00; p01 = n01; p10 = n10; p11 = n11;
    }

    /** Rumo estimado (rad). */
    public double getRumo() {
        return rumo;
    }

    /** Velocidade estimada do rumo (rad/s). */
    public double getVelocidade() {
        return velocidade;
    }
}
