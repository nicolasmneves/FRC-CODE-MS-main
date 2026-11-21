package frc.robot.subsystems.Secondaries.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// ============ SUBSISTEMA DE LEDS =================
// Fita endereçável (WS2812/NeoPixel) ligada direto numa porta PWM do roboRIO.
// O roboRIO só aceita UMA fita AddressableLED: para mais de uma fita, ligue em série
// e aumente LED_COUNT.
//
// Uso: um comando define a cor (ou padrão) e o periodic() envia o buffer para a fita.
public class LedSubsystem extends SubsystemBase
{
    // ============ CONFIGURAÇÃO =================
    public static final int PWM_PORT = 9;      // CONFERIR: porta PWM onde o fio de dados está ligado
    public static final int LED_COUNT = 60;    // CONFERIR: quantidade de LEDs da fita
    public static final double BRIGHTNESS = 0.6; // brilho de 0 a 1

    // Cor de cada modo (LB = Torre, RB = Matriz).
    public static final Color TORRE_COLOR  = Color.kGreen;
    public static final Color MATRIZ_COLOR = Color.kRed;

    private final AddressableLED led = new AddressableLED(PWM_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_COUNT);

    public LedSubsystem()
    {
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic()
    {
        // Envia o que os comandos escreveram no buffer neste ciclo.
        led.setData(buffer);
    }

    // ============ APLICAR UM PADRÃO =================
    private void apply(LEDPattern pattern)
    {
        pattern.atBrightness(Percent.of(BRIGHTNESS * 100.0)).applyTo(buffer);
    }

    // ============ COMANDOS =================

    /** Mantém a fita na cor dada pelo supplier (reavaliada a cada ciclo). Funciona com o robô desabilitado. */
    public Command followColorCommand(Supplier<Color> color)
    {
        return this.run(() -> apply(LEDPattern.solid(color.get())))
            .ignoringDisable(true)
            .withName("LED cor");
    }

    /** Igual ao followColorCommand, mas pisca rápido enquanto piscar.getAsBoolean() for true. */
    public Command followColorCommand(Supplier<Color> color, BooleanSupplier piscar)
    {
        return this.run(() -> {
                LEDPattern base = LEDPattern.solid(color.get());
                apply(piscar.getAsBoolean() ? base.blink(Seconds.of(0.1)) : base);
            })
            .ignoringDisable(true)
            .withName("LED cor");
    }

    /** Cor fixa enquanto o comando roda. */
    public Command solidCommand(Color color)
    {
        LEDPattern pattern = LEDPattern.solid(color);
        return this.run(() -> apply(pattern)).ignoringDisable(true).withName("LED " + color);
    }

    /** Apaga a fita. */
    public Command offCommand()
    {
        return this.run(() -> LEDPattern.kOff.applyTo(buffer)).ignoringDisable(true).withName("LED apagado");
    }
}
