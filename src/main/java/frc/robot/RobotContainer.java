package frc.robot;

import java.io.File;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.drive.tuning.HeadingTuningCommand;
import frc.robot.commands.drive.tuning.SteerTuningCommand;
import frc.robot.commands.vision.AproximarComMira;
import frc.robot.commands.vision.ComandosVisao;
import frc.robot.commands.vision.SeguirAlvo;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.Drive.assist.Freio;
import frc.robot.subsystems.Drive.assist.LimitadorGiro;
import frc.robot.subsystems.Drive.assist.LimiteFrenteLidar;
import frc.robot.subsystems.Drive.swerve.SwerveSubsystem;
import frc.robot.subsystems.Mechanisms.elevator.ElevatorSubsystem;
import frc.robot.subsystems.Mechanisms.intake.GarraIntakeSubsystem;
import frc.robot.subsystems.Secondaries.leds.LedSubsystem;

import swervelib.SwerveInputStream;

// ============ ROBOT CONTAINER ============
// Monta o robô: cria os subsistemas, liga os botões aos comandos e escolhe o autônomo.
//
// MAPA DO CONTROLE (GameSir Tarantula Pro, visto como Xbox na porta 0)
//   Analógico esq. .. andar            | Analógico dir. .. rumo do robô
//   LT ............. intake + freio    | RT ............. outtake
//   LB ............. modo TORRE        | RB ............. modo MATRIZ
//   A .............. elevador desce    | X / Y / B ...... pontuar nível 1 / 2 / 3
//   D-pad ↑ / ↓ .... elevador manual   | Start (C3) ..... rezerar rumo
//   Pá esq. (D-pad ←) identificar cubo | Back (C4) ...... zerar elevador
//   Pá dir. (D-pad →) ir ao cubo e coletar
public class RobotContainer {

        // ####################################################################
        // 1. CONTROLE
        // ####################################################################
        private final CommandXboxController m_Controller = new CommandXboxController(0);

        // ####################################################################
        // 2. SUBSISTEMAS E SENSORES
        // ####################################################################

        // Câmera, LIDAR, sensor da base e dashboard (ver Primario).
        public final Primario primario = new Primario();

        // ---- Mecanismos ----
        public final ElevatorSubsystem m_Elevator = new ElevatorSubsystem();
        public final GarraIntakeSubsystem m_intake = new GarraIntakeSubsystem();

        // ---- Secundários ----
        public final LedSubsystem m_Leds = new LedSubsystem(); // verde = Torre, vermelho = Matriz

        // ---- Drive ----
        // Lê a configuração dos módulos em deploy/swerve/neo e configura o PathPlanner.
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve/neo"));

        // Suaviza a aceleração do giro (compartilhado com o tuning de rumo).
        private final LimitadorGiro limitadorGiro = new LimitadorGiro(
                        Constants.DrivebaseConstants.GIRO_SUBIDA, Constants.DrivebaseConstants.GIRO_DESCIDA);

        // LIDAR na frente: não deixa passar da distância da Torre/Matriz e confere o alinhamento.
        private final LimiteFrenteLidar limiteFrente = new LimiteFrenteLidar(primario.lidar);

        // ####################################################################
        // 3. CONDUÇÃO (ENTRADAS DO SWERVE)
        // ####################################################################

        // Analógico esquerdo = translação (LT freia até 20%); analógico direito X = giro.
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -m_Controller.getLeftY() * Freio.fator(m_Controller.getLeftTriggerAxis(), 0.2),
                        () -> -m_Controller.getLeftX() * Freio.fator(m_Controller.getLeftTriggerAxis(), 0.2))
                        .withControllerRotationAxis(m_Controller::getRightX)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleRotation(.4)
                        .allianceRelativeControl(true);

        // Analógico direito APONTA o rumo do robô (condução padrão).
        // sinalAlianca(): o YAGSL gira só o analógico esquerdo no vermelho; aqui giramos o direito.
        public final SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(
                                        () -> -m_Controller.getRightX() * sinalAlianca(),
                                        () -> -m_Controller.getRightY() * sinalAlianca())
                        .headingWhile(true)
                        .scaleRotation(6)
                        .scaleTranslation(.6);

        // ####################################################################
        // 4. CONFIGURAÇÕES DO JOGO (FÁCEIS DE MUDAR)
        // ####################################################################

        // ---- Arena ECOSYNC (14 x 6 m), para espelhar os paths no vermelho ----
        private static final double FIELD_LENGTH_METERS = 14.0;
        private static final double FIELD_WIDTH_METERS = 6.0;

        // ---- Alturas do elevador (cm) para A, X, Y, B em cada modo ----
        private static final double[] ALTURAS_TORRE = {0, 64, 88, 104};
        private static final double[] ALTURAS_MATRIZ = {0, 35, 64, 100};

        // ---- Garra ----
        private static final double[] RPM_TORRE = {600, 500, 600, 600}; // soltar na Torre, por estágio (A, X, Y, B)
        private static final double RPM_MATRIZ = 500;   // soltar na Matriz
        private static final double RPM_COLETA = -1000; // puxar a peça (LT e pá direita)
        private static final double RPM_OUTTAKE = 1000; // expulsar (RT)

        // ---- Tempos da pontuação (s) ----
        private static final double TEMPO_SOLTAR_S = 1.0;      // garra ligada soltando a peça
        private static final double TEMPO_MAX_SUBIDA_S = 11.0; // se o elevador não chegar, solta mesmo assim

        // ---- Visão ----
        private static final String OBJETO_CUBO = "cubo"; // nome cadastrado em VisionProfilesFactory

        // ---- Estado ----
        private boolean modoTorre = false; // false = Matriz (início), true = Torre. Trocado pelos bumpers.

        // ####################################################################
        // 5. CONSTRUTOR
        // ####################################################################
        public RobotContainer() {
                configureFieldSize();
                configureBindings();
                configureAutonomous(); // depois do swerve: o AutoBuilder já precisa existir
                modoTeste.setDefault(0); // publica /Tuning/Mode desde o boot

                DriverStation.silenceJoystickConnectionWarning(true);
        }

        // Tamanho real da arena para o PathPlanner (o padrão dele é o campo oficial da FIRST).
        // Rotacional = vermelho é o azul girado 180°. Se for espelhado, use FieldSymmetry.kMirrored.
        private void configureFieldSize() {
                FlippingUtil.fieldSizeX = FIELD_LENGTH_METERS;
                FlippingUtil.fieldSizeY = FIELD_WIDTH_METERS;
                FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;
        }

        // ####################################################################
        // 6. BOTÕES
        // ####################################################################
        private void configureBindings() {
                configurarConducao();
                configurarModo();
                configurarElevador();
                configurarGarra();
                configurarVisao();
                configurarSistema();
        }

        // ---- Condução: giro suavizado + limite da frente pelo LIDAR ----
        private void configurarConducao() {
                drivebase.setDefaultCommand(drivebase.driveFieldOriented(() -> {
                        ChassisSpeeds v = driveDirectAngle.get();
                        v.omegaRadiansPerSecond = limitadorGiro.calcular(v.omegaRadiansPerSecond);
                        return limiteFrente.limitar(v, drivebase.getHeading(), modoTorre);
                }));
        }

        // ---- Modo Torre / Matriz (define as alturas de A, X, Y, B) + LEDs ----
        private void configurarModo() {
                m_Controller.leftBumper().onTrue(Commands.runOnce(() -> modoTorre = true).ignoringDisable(true));
                m_Controller.rightBumper().onTrue(Commands.runOnce(() -> modoTorre = false).ignoringDisable(true));

                // Cor do modo o tempo todo; pisca quando está de frente e na distância (LIDAR).
                m_Leds.setDefaultCommand(m_Leds.followColorCommand(
                                () -> modoTorre ? LedSubsystem.TORRE_COLOR : LedSubsystem.MATRIZ_COLOR,
                                limiteFrente::isPronto));
        }

        // ---- Elevador: A desce, X/Y/B pontuam, D-pad manual ----
        // Apertar outro botão de altura no meio cancela a sequência anterior.
        private void configurarElevador() {
                m_Controller.a().onTrue(alturaDoModo(0));
                m_Controller.x().onTrue(pontuar(1));
                m_Controller.y().onTrue(pontuar(2));
                m_Controller.b().onTrue(pontuar(3));

                m_Controller.povUp().whileTrue(m_Elevator.upCommand());
                m_Controller.povDown().whileTrue(m_Elevator.downCommand());
        }

        // ---- Garra: LT puxa, RT expulsa; solto = parada ----
        private void configurarGarra() {
                m_Controller.leftTrigger().whileTrue(m_intake.setRPM(RPM_COLETA));
                m_Controller.leftTrigger().whileFalse(m_intake.setRPM(0));

                m_Controller.rightTrigger().whileTrue(m_intake.setRPM(RPM_OUTTAKE));
                m_Controller.rightTrigger().whileFalse(m_intake.setRPM(0));
        }

        // ---- Visão: pás traseiras (mapeadas no app GameSir para D-pad ← e →) ----
        private void configurarVisao() {
                m_Controller.povLeft().whileTrue(identificarCubo());
                m_Controller.povRight().whileTrue(irAoCuboEColetar());
                // O setRPM do YAMS mantém o último setpoint: zera ao soltar a pá.
                m_Controller.povRight().onFalse(m_intake.setRPM(0).withTimeout(0.1));
        }

        // ---- Sistema: rezerar rumo (Start) e zerar elevador (Back) ----
        private void configurarSistema() {
                // Robô de frente para LONGE do piloto: rumo vira 0° (azul) ou 180° (vermelho).
                m_Controller.start().onTrue(
                                Commands.runOnce(drivebase::zeroGyroWithAlliance).ignoringDisable(true));
                // Elevador todo embaixo: posição atual vira 0 cm (cancela comandos do elevador).
                m_Controller.back().onTrue(
                                Commands.runOnce(m_Elevator::zeroEncoder, m_Elevator).ignoringDisable(true));
        }

        // ####################################################################
        // 7. COMANDOS COMPOSTOS
        // ####################################################################

        // ---- Altura do botão no modo atual (decidido NA HORA do botão) ----
        private Command alturaDoModo(int botao) {
                return Commands.either(
                                m_Elevator.goToHeightCommand(ALTURAS_TORRE[botao]),
                                m_Elevator.goToHeightCommand(ALTURAS_MATRIZ[botao]),
                                () -> modoTorre);
        }

        // ---- Pontuar: sobe -> espera chegar -> solta a peça -> para -> desce ----
        private Command pontuar(int botao) {
                return Commands.either(
                                sequenciaPontuar(ALTURAS_TORRE[botao], RPM_TORRE[botao], ALTURAS_TORRE[0]),
                                sequenciaPontuar(ALTURAS_MATRIZ[botao], RPM_MATRIZ, ALTURAS_MATRIZ[0]),
                                () -> modoTorre);
        }

        private Command sequenciaPontuar(double alturaCm, double rpm, double alturaDescidaCm) {
                return Commands.sequence(
                                m_Elevator.goToHeightAndWaitCommand(alturaCm).withTimeout(TEMPO_MAX_SUBIDA_S),
                                m_intake.setRPM(rpm).withTimeout(TEMPO_SOLTAR_S),
                                m_intake.setRPM(0).withTimeout(0.5),
                                m_Elevator.goToHeightCommand(alturaDescidaCm));
        }

        // ---- Identificar cubo (segurar): a frente trava no cubo, o piloto continua andando ----
        private Command identificarCubo() {
                var cam = primario.camera;
                return Commands.defer(() -> {
                        String seletorAnterior = cam.getSeletorAlvo();
                        return Commands.runOnce(() -> cam.setSeletorAlvo(OBJETO_CUBO))
                                        .andThen(new SeguirAlvo(drivebase, cam, driveDirectAngle))
                                        .finallyDo(() -> cam.setSeletorAlvo(seletorAnterior));
                }, Set.of(drivebase)).withName("Identificar cubo");
        }

        // ---- Ir ao cubo e coletar (segurar) ----
        // Anda sozinho até o cubo; quando o sensor da base vê o cubo, liga o intake.
        // Se perder o cubo no caminho, o intake NÃO liga.
        private Command irAoCuboEColetar() {
                var cam = primario.camera;
                var sensorBase = primario.sensorFesto;
                return Commands.defer(() -> {
                        String seletorAnterior = cam.getSeletorAlvo();
                        return Commands.sequence(
                                        Commands.runOnce(() -> cam.setSeletorAlvo(OBJETO_CUBO)),
                                        new AproximarComMira(drivebase, cam, sensorBase),
                                        Commands.either(m_intake.setRPM(RPM_COLETA), Commands.none(), sensorBase::getSensor))
                                        .finallyDo(() -> cam.setSeletorAlvo(seletorAnterior));
                }, Set.of(drivebase, m_intake)).withName("Ir ao cubo e coletar");
        }

        // ####################################################################
        // 8. AUTÔNOMO
        // ####################################################################

        private SendableChooser<Command> autoChooser;

        // Comandos nomeados ANTES do seletor (o PathPlanner os procura ao montar os autos).
        private void configureAutonomous() {
                NamedCommands.registerCommand("test", Commands.print("Comando nomeado 'test' executado"));

                if (AutoBuilder.isConfigured()) {
                        autoChooser = AutoBuilder.buildAutoChooser();
                } else {
                        autoChooser = new SendableChooser<>();
                        autoChooser.setDefaultOption("Nenhum (PathPlanner nao configurado)", Commands.none());
                }
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        // ####################################################################
        // 9. MODO TEST (TUNING)
        // ####################################################################

        // /Tuning/Mode: 0 = rotação dos módulos (robô suspenso), 1 = rumo do robô (no chão).
        private final DoubleEntry modoTeste = NetworkTableInstance.getDefault()
                        .getDoubleTopic("/Tuning/Mode").getEntry(0);

        public Command getTestCommand() {
                modoTeste.setDefault(0);
                return modoTeste.get() == 1 ? new HeadingTuningCommand(drivebase, limitadorGiro) : getSteerTuningCommand();
        }

        public Command getSteerTuningCommand() {
                var pid = drivebase.getSwerveDrive().getModules()[0].getAnglePIDF();
                return new SteerTuningCommand(drivebase, pid.p, pid.d, pid.f, Constants.DrivebaseConstants.STEER_KV);
        }

        // ####################################################################
        // 10. TELEMETRIA (chamado a cada ciclo pelo Robot.robotPeriodic)
        // ####################################################################
        public void publicarPose() {
                // ---- Pose ----
                SmartDashboard.putNumber("pose robox ", drivebase.getPose().getX());
                SmartDashboard.putNumber("pose roboy ", drivebase.getPose().getY());
                SmartDashboard.putNumber("pose rotacao ", drivebase.getPose().getRotation().getDegrees());

                // ---- Modo e visão ----
                SmartDashboard.putBoolean("Mira/Vendo objeto", ComandosVisao.vendoAlvo(primario.camera));
                SmartDashboard.putString("Modo", modoTorre ? "TORRE" : "MATRIZ");
                SmartDashboard.putNumber("Modo/Estagio torre", estagioTorreAtual());
                SmartDashboard.putNumber("Modo/RPM torre", RPM_TORRE[estagioTorreAtual()]);
                limiteFrente.publicar(drivebase.getHeading(), modoTorre);

                // Republica o modo de teste para não voltar a 0 quando o notebook desconecta.
                modoTeste.set(modoTeste.get());

                // ---- Rumo pedido pelo analógico direito x rumo atual ----
                double hx = -m_Controller.getRightX() * sinalAlianca();
                double hy = -m_Controller.getRightY() * sinalAlianca();
                double alvo = Math.toDegrees(Math.atan2(hx, hy));
                double atual = drivebase.getHeading().getDegrees();
                SmartDashboard.putBoolean("Rumo/Apontando", Math.hypot(hx, hy) > 0.5);
                SmartDashboard.putNumber("Rumo/AlvoGraus", alvo);
                SmartDashboard.putNumber("Rumo/AtualGraus", atual);
                SmartDashboard.putNumber("Rumo/ErroGraus", MathUtil.inputModulus(alvo - atual, -180, 180));
        }

        // Estágio da Torre mais próximo da altura real do elevador.
        private int estagioTorreAtual() {
                double altura = m_Elevator.getHeightCm();
                int maisPerto = 0;
                for (int i = 1; i < ALTURAS_TORRE.length; i++) {
                        if (Math.abs(ALTURAS_TORRE[i] - altura) < Math.abs(ALTURAS_TORRE[maisPerto] - altura)) {
                                maisPerto = i;
                        }
                }
                return maisPerto;
        }

        // ####################################################################
        // 11. UTILITÁRIOS (USADOS PELO Robot.java)
        // ####################################################################

        // -1 no vermelho (gira o rumo pedido em 180°), +1 no azul ou sem aliança.
        private static double sinalAlianca() {
                var alianca = DriverStation.getAlliance();
                return alianca.isPresent() && alianca.get() == DriverStation.Alliance.Red ? -1.0 : 1.0;
        }

        // Desabilitado: quando a aliança aparece/muda, rumo = 0° (azul) ou 180° (vermelho).
        private DriverStation.Alliance ultimaAlianca = null;

        public void alinharRumoComAlianca() {
                var alianca = DriverStation.getAlliance();
                if (alianca.isPresent() && alianca.get() != ultimaAlianca) {
                        ultimaAlianca = alianca.get();
                        drivebase.zeroGyroWithAlliance();
                }
        }

        // Freio (brake) ou solto (coast) nos motores de tração.
        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}
