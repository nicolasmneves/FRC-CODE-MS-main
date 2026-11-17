package frc.robot.Codigo.Visao;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Pede uma foto ao PC. Cada disparo soma 1 no contador "Visao/Foto/Disparo";
 * o script SalvarFotosPC.py (no PC) ve a mudanca e salva o quadro do stream.
 */
public final class FotoPC {

    private static final String K_DISPARO = "Visao/Foto/Disparo";
    private static int contador = 0;

    private FotoPC() {
    }

    public static void disparar() {
        SmartDashboard.putNumber(K_DISPARO, ++contador);
    }
}
