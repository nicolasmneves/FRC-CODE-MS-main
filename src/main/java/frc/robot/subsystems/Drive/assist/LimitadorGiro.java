package frc.robot.subsystems.Drive.assist;

import edu.wpi.first.wpilibj.Timer;

// ============ LIMITADOR DE ACELERAÇÃO DO GIRO ============
// Suaviza a velocidade angular pedida ao chassi. Sem ele, o controle de rumo (analógico direito)
// pede a velocidade máxima de uma vez: os 4 motores de tração puxam pico de corrente juntos, a
// bateria afunda e o giro sai aos trancos.
// - "subida": quanto |ω| pode AUMENTAR por segundo (rad/s²). É o que corta o pico de corrente.
// - "descida": quanto |ω| pode DIMINUIR por segundo (rad/s²). Mais alto, para frear perto do alvo
//   sem passar dele.
// - Inverter o sentido passa por zero respeitando os dois limites.
public class LimitadorGiro
{
    private double subida, descida;
    private double ultimo = 0;
    private double ultimoTempo = -1;

    public LimitadorGiro(double subidaRadS2, double descidaRadS2)
    {
        this.subida = subidaRadS2;
        this.descida = descidaRadS2;
    }

    public void configurar(double subidaRadS2, double descidaRadS2)
    {
        this.subida = subidaRadS2;
        this.descida = descidaRadS2;
    }

    public double calcular(double pedido)
    {
        double agora = Timer.getFPGATimestamp();
        double dt = ultimoTempo < 0 ? 0.02 : Math.min(agora - ultimoTempo, 0.1);
        ultimoTempo = agora;

        double saida;
        boolean mesmoSentido = Math.signum(pedido) == Math.signum(ultimo) || ultimo == 0;
        if (mesmoSentido && Math.abs(pedido) >= Math.abs(ultimo))
        {
            // acelerando no mesmo sentido: limita pela subida
            saida = ultimo + Math.signum(pedido) * Math.min(Math.abs(pedido) - Math.abs(ultimo), subida * dt);
        }
        else
        {
            // freando (ou invertendo): primeiro reduz |ω| pela descida
            double alvo = mesmoSentido ? pedido : 0;
            double passo = Math.min(Math.abs(ultimo) - Math.abs(alvo), descida * dt);
            saida = ultimo - Math.signum(ultimo) * passo;
        }
        ultimo = saida;
        return saida;
    }

    public void reset(double valor)
    {
        ultimo = valor;
        ultimoTempo = -1;
    }
}
