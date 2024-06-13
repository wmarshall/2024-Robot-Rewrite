// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PortConstants;
import frc.robot.subsystems.climber.ClimberHardware;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberConstants;
import frc.robot.subsystems.indexer.IndexerHardware;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeHardware;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RobotContainer {

  private final ClimberSubsystem climber = new ClimberSubsystem(new ClimberHardware());
  private final IntakeSubsystem intake = new IntakeSubsystem(new IntakeHardware());
  private final IndexerSubsystem indexer = new IndexerSubsystem(new IndexerHardware());

  private final CommandFactory commandFactory = new CommandFactory(indexer, intake);

  private final CommandXboxController driverController = new CommandXboxController(PortConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(PortConstants.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    configureDefaultCommands();
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDefaultCommands() {
    climber.setDefaultCommand(climber.setConcurrentSpeed(0));
    indexer.setDefaultCommand(indexer.runIndexer(0));
    intake.setDefaultCommand(intake.runIntake(0));
  }

  private void configureDriverBindings() {
    driverController.rightTrigger().whileTrue(commandFactory.automaticIntake());
  }

  private void configureOperatorBindings() {
    operatorController.rightTrigger().whileTrue(climber.setRightSpeed(-ClimberConstants.CLIMBER_SPEED_SINGLE));
    operatorController.leftTrigger().whileTrue(climber.setLeftSpeed(-ClimberConstants.CLIMBER_SPEED_SINGLE));
    operatorController.rightBumper().whileTrue(climber.setRightSpeed(ClimberConstants.CLIMBER_SPEED_SINGLE));
    operatorController.leftBumper().whileTrue(climber.setLeftSpeed(ClimberConstants.CLIMBER_SPEED_SINGLE));
    operatorController.b().whileTrue(climber.setConcurrentSpeed(ClimberConstants.CLIMBER_SPEED_CONCURRENT));
    operatorController.a().whileTrue(climber.setConcurrentSpeed(-ClimberConstants.CLIMBER_SPEED_CONCURRENT));
  }
}
