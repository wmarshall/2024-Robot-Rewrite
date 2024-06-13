// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PortConstants;
import frc.robot.subsystems.climber.ClimberHardware;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberConstants;

public class RobotContainer {

  private final ClimberSubsystem climber = new ClimberSubsystem(new ClimberHardware());

  private final CommandXboxController driverController = new CommandXboxController(PortConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(PortConstants.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {

  }

  private void configureOperatorBindings() {
   operatorController.leftTrigger().whileTrue(climber.setLeftSpeed(-ClimberConstants.CLIMBER_SPEED_SINGLE));
   operatorController.rightTrigger().whileTrue(climber.setRightSpeed(-ClimberConstants.CLIMBER_SPEED_SINGLE));
   operatorController.leftBumper().whileTrue(climber.setLeftSpeed(ClimberConstants.CLIMBER_SPEED_SINGLE));
   operatorController.rightBumper().whileTrue(climber.setRightSpeed(ClimberConstants.CLIMBER_SPEED_SINGLE));
   operatorController.a().whileTrue(climber.setConcurrentSpeed(-ClimberConstants.CLIMBER_SPEED_CONCURRENT));
   operatorController.b().whileTrue(climber.setConcurrentSpeed(ClimberConstants.CLIMBER_SPEED_CONCURRENT));
  }
}
