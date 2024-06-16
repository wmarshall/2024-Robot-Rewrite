package frc.robot;

import frc.robot.Constants.MotorIdConstants;
import frc.robot.subsystems.climber.ClimberHardware;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroPigeon;
import frc.robot.subsystems.drive.GyroSim;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleHardware;
import frc.robot.subsystems.drive.SwerveModuleSim;
import frc.robot.subsystems.drive.SwerveModule.SwerveModuleConstants;
import frc.robot.subsystems.indexer.IndexerHardware;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeHardware;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterHardware;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotSubsystemFactory {
    private final boolean isSim;

    public RobotSubsystemFactory(boolean isSim) {
        this.isSim = isSim;
    }

    protected IntakeSubsystem buildIntake() {
        return new IntakeSubsystem(new IntakeHardware());
    }

    protected IndexerSubsystem buildIndexer() {
        return new IndexerSubsystem(new IndexerHardware());
    }

    protected ShooterSubsystem buildShooter() {
        return new ShooterSubsystem(new ShooterHardware());
    }

    protected ClimberSubsystem buildClimber() {
        return new ClimberSubsystem(new ClimberHardware());
    }

    protected DriveSubsystem buildDriveTrain() {
        if (isSim) {
            return new DriveSubsystem(
                    new SwerveModule(new SwerveModuleSim("front left")),
                    new SwerveModule(new SwerveModuleSim("front right")),
                    new SwerveModule(new SwerveModuleSim("rear left")),
                    new SwerveModule(new SwerveModuleSim("rear right")), new GyroSim());
        } else {
            return new DriveSubsystem(
                    new SwerveModule(new SwerveModuleHardware(MotorIdConstants.FRONT_LEFT_DRIVE_CAN_ID,
                            MotorIdConstants.FRONT_LEFT_STEER_CAN_ID,
                            SwerveModuleConstants.kFrontLeftChassisAngularOffset,
                            "front left")),
                    new SwerveModule(new SwerveModuleHardware(MotorIdConstants.FRONT_RIGHT_DRIVE_CAN_ID,
                            MotorIdConstants.FRONT_RIGHT_STEER_CAN_ID,
                            SwerveModuleConstants.kFrontRightChassisAngularOffset,
                            "front right")),
                    new SwerveModule(new SwerveModuleHardware(MotorIdConstants.REAR_LEFT_DRIVE_CAN_ID,
                            MotorIdConstants.REAR_LEFT_STEER_CAN_ID,
                            SwerveModuleConstants.kRearLeftChassisAngularOffset,
                            "rear left")),
                    new SwerveModule(new SwerveModuleHardware(MotorIdConstants.REAR_RIGHT_DRIVE_CAN_ID,
                            MotorIdConstants.REAR_RIGHT_STEER_CAN_ID,
                            SwerveModuleConstants.kRearRightChassisAngularOffset,
                            "rear right")),
                    new GyroPigeon());
        }
    }
}
