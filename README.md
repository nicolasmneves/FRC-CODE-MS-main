# FRC-CODE-MS

Código do robô da equipe **MS** para a FIRST Robotics Competition (FRC), feito em Java com WPILib 2026.

## O que tem aqui

- **Swerve drive** com [YAGSL](https://github.com/BroncBotz3481/YAGSL): configurações prontas para módulos NEO, Falcon e MAXSwerve em `src/main/deploy/swerve/`
- **Mecanismos**: garra/intake e elevador (`subsystems/Mechanisms`)
- **Autônomo** com PathPlanner (`src/main/deploy/pathplanner/`)
- **Visão HSV** com OpenCV numa câmera USB no roboRIO, com controles e stream no Elastic (`Codigo/Visao`)
- **Ajustes de mira e aproximação** (filtro de Kalman, seguir alvo, distância da parede)
- **Sensores**: LIDAR-Lite v3 e sensor Festo SOEG

## Tecnologias

Java 17 · WPILib/GradleRIO 2026 · YAGSL · PathPlannerLib · Phoenix 6 (CTRE) · REVLib · PhotonLib · YAMS · maple-sim

## Como rodar

1. Instale o [WPILib 2026](https://docs.wpilib.org/) (ele já traz o VS Code configurado).
2. Abra a pasta do projeto no **WPILib VS Code**.
3. Compile com `./gradlew build`.
4. Com o robô conectado, faça o deploy com `./gradlew deploy`. Também dá para usar a paleta do WPILib (`Deploy Robot Code`).
5. Para testar sem robô, rode `./gradlew simulateJava`.

## Estrutura

```
src/main/java/frc/robot/
├── Robot.java / RobotContainer.java
├── commands/swervedrive/     comandos de direção e autobalance
├── subsystems/               swerve, garra e elevador
├── constants/                constantes do robô e do operador
└── Codigo/                   visão, dashboard, sensores e ajustes de mira
src/main/deploy/              swerve, pathplanner e perfis HSV
```

O guia completo da visão HSV está em [`LEIA-ME_VISAO.txt`](LEIA-ME_VISAO.txt).
