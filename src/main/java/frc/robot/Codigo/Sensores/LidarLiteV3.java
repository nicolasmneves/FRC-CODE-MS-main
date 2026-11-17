package frc.robot.Codigo.Sensores;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

/**
 * Garmin LIDAR-Lite v3 via I2C (endereco 0x62).
 *
 * Ligacao (roboRIO):
 *   Vermelho 5V   -> 5V (o conector I2C do roboRIO so tem 3.3V: use 5V de DIO/PWM/MXP)
 *   Preto    GND  -> GND
 *   Verde    SCL  -> SCL da porta I2C
 *   Azul     SDA  -> SDA da porta I2C
 *   Laranja/Amarelo -> sem ligar
 *   Recomendado: capacitor 680 uF entre 5V e GND perto do sensor.
 *
 * Uso: init() uma vez, periodic() a cada ciclo (20 ms). Nao trava o loop:
 * dispara a medida num ciclo e le no seguinte (~25-50 leituras/s).
 * O sensor entrega cm; aqui ja sai convertido para mm.
 */
public class LidarLiteV3 {

    public static final int ENDERECO = 0x62;

    private static final int REG_COMANDO = 0x00;
    private static final int REG_STATUS = 0x01;
    /** 0x0f (byte alto da distancia) + bit 7 = le 0x0f e 0x10 em sequencia. */
    private static final int REG_DISTANCIA = 0x8f;
    /** Medir com correcao de vies do receptor. */
    private static final int CMD_MEDIR = 0x04;
    private static final int BIT_OCUPADO = 0x01;

    /** Tempo maximo esperando uma medida (s). */
    private static final double TIMEOUT_MEDIDA = 0.1;
    /** Falhas seguidas para considerar desconectado. */
    private static final int FALHAS_DESCONECTADO = 5;
    /** Intervalo entre tentativas quando desconectado (s). */
    private static final double INTERVALO_RECONEXAO = 0.5;

    private final I2C i2c;
    private final byte[] umByte = new byte[1];
    private final byte[] doisBytes = new byte[2];

    private boolean medindo = false;
    private double inicioMedida = 0;
    private double proximaTentativa = 0;
    private int falhasSeguidas = 0;

    private double distanciaMm = -1;
    private double offsetMm = 0;
    private double timestampLeitura = 0;
    private boolean conectado = false;
    private long leiturasOk = 0;
    private long erros = 0;

    /** Porta I2C onboard do roboRIO. */
    public LidarLiteV3() {
        this(I2C.Port.kOnboard);
    }

    /** @param porta I2C.Port.kOnboard ou I2C.Port.kMXP */
    public LidarLiteV3(I2C.Port porta) {
        i2c = new I2C(porta, ENDERECO);
    }

    /** Chamar uma vez (robotInit). */
    public void init() {
        medindo = false;
        falhasSeguidas = 0;
        conectado = !i2c.addressOnly(); // addressOnly() devolve true se ninguem respondeu
    }

    /** Chamar a cada ciclo (robotPeriodic). */
    public void periodic() {
        double agora = Timer.getFPGATimestamp();

        if (!conectado && agora < proximaTentativa)
            return;

        if (medindo) {
            if (!lerStatus())
                return;
            if ((umByte[0] & BIT_OCUPADO) != 0) {
                // ainda medindo
                if (agora - inicioMedida > TIMEOUT_MEDIDA)
                    falhou(agora);
                return;
            }
            if (!lerDistancia())
                return;
            medindo = false;
        }

        // dispara a proxima medida (lida no proximo ciclo)
        if (i2c.write(REG_COMANDO, CMD_MEDIR)) {
            falhou(agora);
            return;
        }
        medindo = true;
        inicioMedida = agora;
    }

    /** Le o registrador de status em umByte. false se falhou. */
    private boolean lerStatus() {
        umByte[0] = (byte) REG_STATUS;
        if (i2c.writeBulk(umByte, 1) || i2c.readOnly(umByte, 1)) {
            falhou(Timer.getFPGATimestamp());
            return false;
        }
        return true;
    }

    /** Le os 2 bytes da distancia (cm) e converte para mm. false se falhou. */
    private boolean lerDistancia() {
        umByte[0] = (byte) REG_DISTANCIA;
        // escrita e leitura separadas: o sensor nao aceita "repeated start"
        if (i2c.writeBulk(umByte, 1) || i2c.readOnly(doisBytes, 2)) {
            falhou(Timer.getFPGATimestamp());
            return false;
        }
        int cm = ((doisBytes[0] & 0xFF) << 8) | (doisBytes[1] & 0xFF);
        distanciaMm = cm * 10.0 + offsetMm;
        timestampLeitura = Timer.getFPGATimestamp();
        leiturasOk++;
        falhasSeguidas = 0;
        conectado = true;
        return true;
    }

    private void falhou(double agora) {
        erros++;
        medindo = false;
        if (++falhasSeguidas >= FALHAS_DESCONECTADO) {
            conectado = false;
            distanciaMm = -1;
            proximaTentativa = agora + INTERVALO_RECONEXAO;
        }
    }

    /** Distancia em mm. -1 se ainda nao leu ou desconectado. */
    public double getDistanciaMm() {
        return distanciaMm;
    }

    /** Distancia em metros. -1 se invalida. */
    public double getDistanciaM() {
        return distanciaMm < 0 ? -1 : distanciaMm / 1000.0;
    }

    /** true se ha leitura valida e recente (menos de 0.25 s). */
    public boolean temLeitura() {
        return conectado && distanciaMm >= 0 && getIdadeLeitura() < 0.25;
    }

    public boolean isConectado() {
        return conectado;
    }

    /** Segundos desde a ultima leitura boa. */
    public double getIdadeLeitura() {
        return Timer.getFPGATimestamp() - timestampLeitura;
    }

    public long getLeiturasOk() {
        return leiturasOk;
    }

    public long getErros() {
        return erros;
    }

    /** Correcao somada a leitura (mm). Calibre medindo a uma distancia conhecida. */
    public void setOffsetMm(double offsetMm) {
        this.offsetMm = offsetMm;
    }

    public double getOffsetMm() {
        return offsetMm;
    }
}
