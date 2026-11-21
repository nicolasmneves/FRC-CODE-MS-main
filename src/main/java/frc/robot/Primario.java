package frc.robot;

import frc.robot.subsystems.Telemetry.elastic.Dashboard;
import frc.robot.subsystems.Odometry.lidar.LidarLiteV3;
import frc.robot.subsystems.Mechanisms.intake.SensorFestoSOEG;
import frc.robot.subsystems.Odometry.vision.CvCamera;
import frc.robot.subsystems.Odometry.vision.hsv.VisionProfilesFactory;

/**
 * Cria e guarda os sistemas principais do robo (uma instancia de cada).
 *
 * No Robot.java:
 *   robotInit     -> new Primario()
 *   robotInit     -> primario.init()
 *   robotPeriodic -> primario.periodic()
 */
public class Primario {

    // ===== sistemas =====
    public final CvCamera camera;
    public final LidarLiteV3 lidar;
    public final SensorFestoSOEG sensorFesto;
    public final Dashboard dashboard;

    public Primario() {
        camera = new CvCamera();      // camera USB 0, stream "VisaoHSV"
        lidar = new LidarLiteV3();    // I2C onboard, endereco 0x62
        sensorFesto = new SensorFestoSOEG();  // ANALOG IN 3 (ver ligacao na classe)
        dashboard = new Dashboard(camera, lidar, sensorFesto);
    }

    /** Chamar uma vez, no robotInit(). */
    public void init() {
        configurarVisao();
        camera.start();

        lidar.init();
        // lidar.setOffsetMm(0); // ajuste apos calibrar

        // dashboard por ultimo: comeca mostrando a config acima
        dashboard.init();
    }

    /** Chamar a cada ciclo, no robotPeriodic(). */
    public void periodic() {
        lidar.periodic();
        dashboard.atualizar();
    }

    /** Configuracao inicial da visao. Tudo pode ser mudado depois (codigo ou Elastic). */
    private void configurarVisao() {
        camera.setQualidade320();         // captura 320x240 (4:3, sem distorcer)
        camera.setProcessarMetade(true);  // processa 160x120: ~4x mais leve, menos atraso na mira
        camera.setQualidadeStream(320, 240); // stream no Elastic continua em 320x240
        camera.setFormatoYUYV(true);      // sem JPEG: evita "Corrupt JPEG data" desta camera
        camera.setFps(30);                // esta camera so tem modos de 30 fps
        camera.setPassoProcessamento(1);  // processa todos os frames (menor atraso)
        camera.setStreamHabilitado(true);
        camera.setDesenharDeteccoes(true);  // false = stream so com a imagem (sem retangulos)
        camera.setModoAlvo(CvCamera.ModoAlvo.CENTRO);
        // camera.setExposicaoManual(30); // use o mesmo valor da calibracao
        camera.setPerfilManager(VisionProfilesFactory.perfilGeral()); // trocavel no Elastic
    }
}
