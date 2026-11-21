package frc.robot.commands.drive.tuning;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drive.swerve.SwerveSubsystem;
import swervelib.SwerveModule;
import swervelib.parser.PIDFConfig;

// ============ TUNING DO PID DE ROTAÇÃO (KRAKEN X60 / MK4i) ============
// Roda só no modo Test do Driver Station (o RobotContainer agenda no testInit).
// Os 4 módulos ficam com a roda parada (velocidade 0) e o ângulo vem da rede:
//   /Tuning/Steer/SetpointDeg  ângulo pedido para os 4 módulos (graus)
//   /Tuning/Steer/kP kD kS kV   ganhos aplicados ao vivo no Slot0 dos TalonFX de rotação
// Telemetria: /Tuning/Steer/AngleDeg (ângulo medido pelo motor) e
//             /Tuning/Steer/ErrorDeg (erro de cada módulo), ordem FL, FR, BL, BR.
// Unidades (YAGSL + Phoenix 6): posição em ROTAÇÕES DO MÓDULO, então kP é volts por
// rotação de erro (kP 100 = 0,28 V por grau) e kV é volts por rotação/s do módulo.
// Depois de achar os ganhos, copie kP/kD/kS para deploy/swerve/neo/modules/pidfproperties.json
// (o campo "f" do YAGSL vira kS) e o kV para Constants.DrivebaseConstants.STEER_KV.
public class SteerTuningCommand extends Command
{
    private final SwerveSubsystem drivebase;
    private final DoubleEntry setpointDeg, kP, kD, kS, kV, supplyLimit, statorLimit, mmCruise, mmAccel;
    private double lastSupply = Double.NaN, lastStator = Double.NaN, lastCruise = Double.NaN, lastAccel = Double.NaN;
    private final DoubleArrayPublisher anglePub, errorPub;
    private double lastP = Double.NaN, lastD = Double.NaN, lastS = Double.NaN, lastV = Double.NaN;

    public SteerTuningCommand(SwerveSubsystem drivebase, double p, double d, double s, double v)
    {
        this.drivebase = drivebase;
        NetworkTable t = NetworkTableInstance.getDefault().getTable("Tuning/Steer");
        setpointDeg = t.getDoubleTopic("SetpointDeg").getEntry(0);
        kP = t.getDoubleTopic("kP").getEntry(p);
        kD = t.getDoubleTopic("kD").getEntry(d);
        kS = t.getDoubleTopic("kS").getEntry(s);
        kV = t.getDoubleTopic("kV").getEntry(v);
        // publica os valores iniciais para aparecerem no dashboard
        setpointDeg.setDefault(0);
        kP.setDefault(p);
        kD.setDefault(d);
        kS.setDefault(s);
        kV.setDefault(v);
        // Limites de corrente da rotação (A). O YAGSL só aplica o "currentLimit.angle" do JSON como
        // limite de SUPPLY; um supply baixo corta o torque de frenagem e o módulo passa do alvo.
        supplyLimit = t.getDoubleTopic("SupplyLimitA").getEntry(Constants.DrivebaseConstants.STEER_SUPPLY_LIMIT);
        statorLimit = t.getDoubleTopic("StatorLimitA").getEntry(Constants.DrivebaseConstants.STEER_STATOR_LIMIT);
        supplyLimit.setDefault(Constants.DrivebaseConstants.STEER_SUPPLY_LIMIT);
        statorLimit.setDefault(Constants.DrivebaseConstants.STEER_STATOR_LIMIT);
        // Perfil Motion Magic da rotação (rot/s e rot/s² do MÓDULO). O YAGSL usa 4,67 e 46,7,
        // mais rápido do que o módulo acompanha; o atraso vira overshoot.
        mmCruise = t.getDoubleTopic("MMCruiseRps").getEntry(Constants.DrivebaseConstants.STEER_MM_CRUISE);
        mmAccel = t.getDoubleTopic("MMAccelRps2").getEntry(Constants.DrivebaseConstants.STEER_MM_ACCEL);
        mmCruise.setDefault(Constants.DrivebaseConstants.STEER_MM_CRUISE);
        mmAccel.setDefault(Constants.DrivebaseConstants.STEER_MM_ACCEL);
        anglePub = t.getDoubleArrayTopic("AngleDeg").publish();
        errorPub = t.getDoubleArrayTopic("ErrorDeg").publish();
        addRequirements(drivebase);
    }

    @Override
    public void initialize()
    {
        lastP = lastD = lastS = lastV = Double.NaN; // força reaplicar os ganhos
        lastSupply = lastStator = Double.NaN;
        lastCruise = lastAccel = Double.NaN;
    }

    @Override
    public void execute()
    {
        SwerveModule[] modules = drivebase.getSwerveDrive().getModules();

        double p = kP.get(), d = kD.get(), s = kS.get(), v = kV.get();
        if (p != lastP || d != lastD || s != lastS || v != lastV)
        {
            for (SwerveModule m : modules)
            {
                aplicarGanhos(m, p, d, s, v);
            }
            lastP = p; lastD = d; lastS = s; lastV = v;
        }
        double sup = supplyLimit.get(), sta = statorLimit.get();
        if (sup != lastSupply || sta != lastStator)
        {
            for (SwerveModule m : modules)
            {
                aplicarLimites(m, sup, sta);
            }
            lastSupply = sup; lastStator = sta;
        }
        double cru = mmCruise.get(), acc = mmAccel.get();
        if (cru != lastCruise || acc != lastAccel)
        {
            for (SwerveModule m : modules)
            {
                aplicarPerfil(m, cru, acc);
            }
            lastCruise = cru; lastAccel = acc;
        }

        double alvo = setpointDeg.get();
        double[] ang = new double[modules.length];
        double[] err = new double[modules.length];
        for (int i = 0; i < modules.length; i++)
        {
            SwerveModule m = modules[i];
            // Otimização do YAGSL continua ligada (desligá-la também desliga o wrap contínuo do PID);
            // por isso os degraus de teste devem ser menores que 90° (ex.: 0° <-> 80°).
            m.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(alvo)), true, true);
            ang[i] = m.getAngleMotor().getPosition() * 360.0; // TalonFX devolve rotações do módulo
            err[i] = Rotation2d.fromDegrees(alvo).minus(Rotation2d.fromDegrees(ang[i])).getDegrees();
        }
        anglePub.set(ang);
        errorPub.set(err);
    }

    // ============ APLICA kP/kD/kS/kV NO TALONFX DE ROTAÇÃO ============
    // Usado aqui e no construtor do SwerveSubsystem (para o kV, que o JSON do YAGSL não tem).
    public static void aplicarGanhos(SwerveModule m, double p, double d, double s, double v)
    {
        m.setAnglePIDF(new PIDFConfig(p, 0, d, s));
        aplicarKV(m, v);
    }

    // Limites de corrente (supply e stator) do TalonFX de rotação.
    public static void aplicarLimites(SwerveModule m, double supplyA, double statorA)
    {
        if (m.getAngleMotor().getMotor() instanceof TalonFX talon)
        {
            CurrentLimitsConfigs lim = new CurrentLimitsConfigs();
            talon.getConfigurator().refresh(lim);
            talon.getConfigurator().apply(lim.withSupplyCurrentLimit(supplyA).withSupplyCurrentLimitEnable(true)
                                             .withStatorCurrentLimit(statorA).withStatorCurrentLimitEnable(true));
        }
    }

    // Perfil Motion Magic (cruzeiro em rot/s e aceleração em rot/s² do módulo).
    public static void aplicarPerfil(SwerveModule m, double cruiseRps, double accelRps2)
    {
        if (m.getAngleMotor().getMotor() instanceof TalonFX talon)
        {
            MotionMagicConfigs mm = new MotionMagicConfigs();
            talon.getConfigurator().refresh(mm);
            talon.getConfigurator().apply(mm.withMotionMagicCruiseVelocity(cruiseRps)
                                            .withMotionMagicAcceleration(accelRps2));
        }
    }

    public static void aplicarKV(SwerveModule m, double v)
    {
        if (m.getAngleMotor().getMotor() instanceof TalonFX talon)
        {
            Slot0Configs slot0 = new Slot0Configs();
            talon.getConfigurator().refresh(slot0);
            talon.getConfigurator().apply(slot0.withKV(v));
        }
    }

    @Override
    public boolean runsWhenDisabled()
    {
        return false;
    }
}
