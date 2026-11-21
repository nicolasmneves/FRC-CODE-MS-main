package frc.robot.subsystems.Mechanisms.intake;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Festo SOEG-RT-M18-PA-K-2L (547912): sensor optico, saida PNP, alimentado com 12 V.
 * Lido pela entrada ANALOGICA 3 do roboRIO (sem resistor):
 *   marrom = 12 V do VRM | azul = GND comum | preto = pino S do ANALOG IN 3.
 * Sem objeto ~0 V, com objeto ~5 V (maximo da leitura).
 */
public class SensorFestoSOEG {

    /** Entrada analogica do sinal. */
    public static final int PORTA_ANALOGICA = 3;
    /** Acima desta tensao = objeto detectado (V). */
    public static final double LIMIAR_VOLTS = 3.5;

    private final AnalogInput sensor = new AnalogInput(PORTA_ANALOGICA);

    /** true = objeto detectado. */
    public boolean getSensor() {
        return sensor.getVoltage() > LIMIAR_VOLTS;
    }

    /** Tensao lida (V), para conferir no Elastic. */
    public double getTensao() {
        return sensor.getVoltage();
    }
}
