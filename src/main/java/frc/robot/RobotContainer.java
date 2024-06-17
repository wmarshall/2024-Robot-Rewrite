// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberConstants;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterConstants;

public class RobotContainer {
  private final RobotSubsystemFactory subsystemFactory = new RobotSubsystemFactory(RobotBase.isSimulation());

  private final ClimberSubsystem climber = subsystemFactory.buildClimber();
  private final IntakeSubsystem intake = subsystemFactory.buildIntake();
  private final IndexerSubsystem indexer = subsystemFactory.buildIndexer();
  private final ShooterSubsystem shooter = subsystemFactory.buildShooter();

  private final CommandFactory commandFactory = new CommandFactory(indexer, intake, shooter);

  private final CommandXboxController driverController = new CommandXboxController(
      DriveControlConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(
      DriveControlConstants.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    configureDefaultCommands();
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDefaultCommands() {
    climber.setDefaultCommand(climber.setConcurrentSpeed(0));
    indexer.setDefaultCommand(indexer.runIndexer(0));
    intake.setDefaultCommand(intake.runIntake(0));
    shooter.setDefaultCommand(shooter.setShootSpeed(0));
  }

  private void configureDriverBindings() {
    driverController.rightTrigger().whileTrue(commandFactory.automaticIntake());
    driverController.leftBumper().whileTrue(shooter.setShootSpeed(ShooterConstants.SHOOTER_SPEED));
    driverController.rightBumper().onTrue(commandFactory.shootWhenUpToSpeed(ShooterConstants.SHOOTER_SPEED));
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
