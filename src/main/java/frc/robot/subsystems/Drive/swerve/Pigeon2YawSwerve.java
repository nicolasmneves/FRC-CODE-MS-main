package frc.robot.subsystems.Drive.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import swervelib.imu.Pigeon2Swerve;

// ============ PIGEON 2 USANDO SÓ O YAW DIRETO ============
// O Pigeon2Swerve do YAGSL calcula o rumo a partir da orientação 3D (quatérnion) do Pigeon. Neste
// robô isso deu o SENTIDO ERRADO (giro horário aumentava o rumo), e o "invertedIMU" do JSON não
// mudava nada. Medido em 18/11/2026: o yaw lido direto do Pigeon (getYaw) estava certo em todos
// os testes (horário = negativo). Esta classe entrega ao YAGSL só esse yaw, como rotação pura em Z.
// O zero (offset) e o resto continuam sendo tratados pelo Pigeon2Swerve.
public class Pigeon2YawSwerve extends Pigeon2Swerve
{
    private final StatusSignal<Angle> yaw;

    public Pigeon2YawSwerve(int canId, String canbus)
    {
        super(canId, canbus);
        yaw = ((Pigeon2) getIMU()).getYaw();
    }

    @Override
    public Rotation3d getRawRotation3d()
    {
        return new Rotation3d(0, 0, Math.toRadians(yaw.refresh().getValueAsDouble()));
    }
}
