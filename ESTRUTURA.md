# Estrutura do código (`src/main/java/frc/robot`)

Padrão **Command-Based** do WPILib: *subsystems* são o hardware (um dono por motor/sensor),
*commands* são as ações que usam esses subsystems, e o `RobotContainer` liga botões a comandos.

```
frc/robot/
├── Main.java                 entrada da JVM (não mexer)
├── Robot.java                ciclo de vida: disabled / auto / teleop / test
├── RobotContainer.java       monta o robô: subsystems, botões, autônomo, telemetria
├── Primario.java             cria câmera, LIDAR, sensor da base e dashboard
│
├── constants/                números globais (Constants, DriveToPoseConstants)
│
├── subsystems/
│   ├── Mechanisms/           mecanismos principais
│   │   ├── elevator/         ElevatorSubsystem (2 NEOs, MAXMotion em cm)
│   │   ├── intake/           GarraIntakeSubsystem + SensorFestoSOEG (sensor da base)
│   │   └── test/             MotorNeo (bancada / testes)
│   ├── Drive/                swerve
│   │   ├── swerve/           SwerveSubsystem (YAGSL) + Pigeon2YawSwerve
│   │   └── assist/           ajudas à pilotagem: Freio, LimitadorGiro, LimiteFrenteLidar
│   ├── Odometry/             onde o robô está / o que ele vê
│   │   ├── lidar/            LidarLiteV3 (distância da frente)
│   │   └── vision/           CvCamera, FotoPC
│   │       └── hsv/          perfis e cores da visão (VisionProfilesFactory)
│   ├── Telemetry/            Elastic, AdvantageScope etc.
│   │   └── elastic/          Dashboard
│   └── Secondaries/          coisas visuais / menos críticas
│       └── leds/             LedSubsystem (verde = Torre, vermelho = Matriz)
│
└── commands/
    ├── drive/                comandos do chassi (DriveToPose, AjustarDistanciaParede...)
    │   └── tuning/           tuning no modo Test (SteerTuning, HeadingTuning)
    ├── vision/               SeguirAlvo, AproximarComMira, ComandosVisao
    │   └── aim/              controle da mira (AjusteMira, MiraControlador, FiltroKalmanRumo)
    └── auto/                 comandos de autônomo (AutoBalance)
```

## Onde mudar o quê

| Quero mudar…                         | Arquivo |
|--------------------------------------|---------|
| Botões do controle                   | `RobotContainer.java` → seção 6 |
| Alturas / RPM da pontuação           | `RobotContainer.java` → seção 4 |
| Distância e rumo da Torre/Matriz     | `subsystems/Drive/assist/LimiteFrenteLidar.java` (topo) |
| PID / velocidade do elevador         | `subsystems/Mechanisms/elevator/ElevatorSubsystem.java` |
| Cores da visão                       | `subsystems/Odometry/vision/hsv/VisionProfilesFactory.java` |
| Cores dos LEDs / porta PWM           | `subsystems/Secondaries/leds/LedSubsystem.java` (topo) |

## Fora do código Java

- `src/main/deploy/` — arquivos que vão para o roboRIO (swerve, PathPlanner, faixas HSV)
- `tools/` — scripts que rodam no PC (ex.: `SalvarFotosPC.py`)
- `vendordeps_desativados/` — bibliotecas tiradas do build (ex.: PhotonVision, não usado)
