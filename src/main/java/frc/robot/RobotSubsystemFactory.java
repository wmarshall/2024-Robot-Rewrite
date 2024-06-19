package frc.robot;

import frc.robot.subsystems.climber.ClimberHardware;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberPlacebo;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.indexer.IndexerHardware;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerPlacebo;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeHardware;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakePlacebo;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterHardware;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterPlacebo;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotSubsystemFactory {
    private final boolean isSim;

    public RobotSubsystemFactory(boolean isSim) {
        this.isSim = isSim;
    }

    protected IntakeSubsystem buildIntake() {
        IntakeIO impl = isSim ? new IntakePlacebo() : new IntakeHardware();
        return new IntakeSubsystem(impl);
    }

    protected IndexerSubsystem buildIndexer() {
        IndexerIO impl = isSim ? new IndexerPlacebo() : new IndexerHardware();
        return new IndexerSubsystem(impl);
    }

    protected ShooterSubsystem buildShooter() {
        ShooterIO impl = isSim ? new ShooterPlacebo() : new ShooterHardware();
        return new ShooterSubsystem(impl);
    }

    protected ClimberSubsystem buildClimber() {
        ClimberIO impl = isSim ? new ClimberPlacebo() : new ClimberHardware();
        return new ClimberSubsystem(impl);
    }
}
