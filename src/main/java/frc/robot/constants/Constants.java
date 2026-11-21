// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    // kV da rotação (Kraken X60, MK4i 150/7): volts por rotação/s do MÓDULO.
    // O JSON do YAGSL não tem kV; o SwerveSubsystem aplica este valor no Slot0 dos TalonFX de rotação.
    // Teórico: 12 V / (100 rot/s do rotor / 21,43) ≈ 2,57, MAS no Motion Magic do Phoenix 6 o kD age no
    // erro de velocidade e também empurra como feedforward: o efetivo é kV + kD. Medido em 18/11/2026:
    // kV 2,4 + kD 0,5 dava 30-40° de overshoot; kV 1,5 + kD 0,5 (≈2,0 efetivo) deu ~2° de overshoot.
    public static final double STEER_KV = 1.5;

    // Limites de corrente da rotação (A). O YAGSL só aplica o "currentLimit.angle" do JSON como SUPPLY;
    // estes valores sobrescrevem supply e stator dos TalonFX de rotação no construtor do SwerveSubsystem.
    public static final double STEER_SUPPLY_LIMIT = 40;
    public static final double STEER_STATOR_LIMIT = 120;

    // Perfil Motion Magic da rotação, em rot/s e rot/s² do MÓDULO (o YAGSL usa 4,67 e 46,7).
    public static final double STEER_MM_CRUISE = 4.67;
    public static final double STEER_MM_ACCEL = 46.7;

    // Limite de aceleração do GIRO DO CHASSI (rad/s²), aplicado ao analógico direito no teleop.
    // SUBIDA: quanto a velocidade angular pode crescer por segundo (corta o pico de corrente que
    // fazia o giro travar com bateria fraca). DESCIDA: quanto pode cair (frear perto do alvo).
    public static final double GIRO_SUBIDA = 5;
    public static final double GIRO_DESCIDA = 45;

    // Roll de montagem do Pigeon 2 (graus). O Pigeon tinha roll 180 gravado na memória dele, o que
    // invertia o yaw (medido em 18/11/2026: girando no sentido HORÁRIO, visto de cima, o yaw SUBIA).
    // Com roll 180 o Pigeon lê gravidade +1 (yaw "puro"), e aí o invertedIMU do swervedrive.json
    // consegue inverter o yaw. Aplicado no Pigeon pelo SwerveSubsystem a cada inicialização.
    public static final double PIGEON_MOUNT_ROLL = 180;
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 4;
  }

  public static final class MotorTestConstants
  {
    public static final int MOTOR_1_CAN_ID = 31;
    public static final int MOTOR_2_CAN_ID = 32;
    public static final int MOTOR_3_CAN_ID = 33;
    public static final int MOTOR_4_CAN_ID = 34;
  }
}
