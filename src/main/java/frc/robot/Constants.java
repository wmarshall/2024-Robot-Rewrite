// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class PortConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class MotorIdConstants {
    public static final int RIGHT_CLIMBER_MOTOR_ID = 0;
    public static final int LEFT_CLIMBER_MOTOR_ID = 1;
    public static final int RIGHT_INTAKE_CENTERING_MOTOR_ID = 2;
    public static final int LEFT_INTAKE_CENTERING_MOTOR_ID = 3;
    public static final int INTAKE_MOTOR_ID = 4;
    public static final int INDEXER_MOTOR_ID = 5;
    public static final int SHOOTER_TOP_MOTOR_ID = 6;
    public static final int SHOOTER_BOTTOM_MOTOR_ID = 7;
  }

  public static class MotorConstants {
    public static final int NEO550_CURRENT_LIMIT = 20;
    public static final int NEO_CURRENT_LIMIT = 50;
    public static final double MINUTE_TO_SECOND_CONVERSION = 1 / 60.0;
  }
}
