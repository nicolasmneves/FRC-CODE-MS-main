package frc.robot.Codigo.Ajustes;

// ============ FREIO PELO GATILHO ============
// Fator de velocidade: gatilho solto = 1 (velocidade total),
// gatilho todo apertado = minimo. No meio, diminui aos poucos.
public final class Freio {

    private Freio() {
    }

    /**
     * @param gatilho valor do gatilho (0 = solto, 1 = todo apertado)
     * @param minimo  fracao da velocidade com o gatilho todo apertado (ex: 0.2 = 20%)
     */
    public static double fator(double gatilho, double minimo) {
        return 1.0 - gatilho * (1.0 - minimo);
    }
}
