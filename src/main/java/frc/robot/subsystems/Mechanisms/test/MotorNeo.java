package frc.robot.subsystems.Mechanisms.test;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * NEO controlado pelo D-pad: cima = sobe, baixo = desce, solto = para.
 * A velocidade (0..1) vem do slider "Motor Neo/Velocidade" no Elastic.
 */
public class MotorNeo extends SubsystemBase {

    private static final String K_VELOCIDADE = "Motor Neo/Velocidade";
    private static final double VELOCIDADE_INICIAL = 0.0;

    private final SparkMax motorNeo = new SparkMax(31, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMaxConfig motorNeoConfig = new SparkMaxConfig();

    public MotorNeo() {
        motorNeoConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        motorNeo.configure(motorNeoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SmartDashboard.putNumber(K_VELOCIDADE, VELOCIDADE_INICIAL);
        setDefaultCommand(parar());
    }

    /**
     * Setar velocidade do motor
     * 
     * @param speed
     */
    public void SetMotorNeo(double speed) {
        motorNeo.set(MathUtil.clamp(speed, -1, 1));
    }

    public double get() {
        return motorNeo.get();
    }

    /** Gira com a velocidade do slider; sentido = +1 (cima) ou -1 (baixo). */
    public Command mover(double sentido) {
        return run(() -> {
            double v = MathUtil.clamp(SmartDashboard.getNumber(K_VELOCIDADE, VELOCIDADE_INICIAL), 0, 1);
            SetMotorNeo(sentido * v);
        });
    }

    /** Motor parado (comando padrao: roda quando nenhum botao esta apertado). */
    public Command parar() {
        return run(() -> SetMotorNeo(0));
    }
}
