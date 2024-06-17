package frc.robot;

import frc.robot.subsystems.climber.ClimberHardware;
import frc.robot.subsystems.climber.ClimberSubsystem;
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
}
