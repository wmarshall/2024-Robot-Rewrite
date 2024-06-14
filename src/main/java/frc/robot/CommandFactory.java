package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem.IndexerConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class CommandFactory {
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;

    public CommandFactory(IndexerSubsystem indexer, IntakeSubsystem intake, ShooterSubsystem shooter) {
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
    }

    public Command automaticIntake() {
        return Commands.either(
                Commands.parallel(indexer.runIndexer(0), intake.runIntake(0)),
                this.manualIntake(),
                () -> indexer.isBeamBroken());
    }

    public Command manualIntake() {
        return Commands.parallel(
                indexer.runIndexer(IntakeConstants.INTAKE_SPEED),
                intake.runIntake(IntakeConstants.INTAKE_SPEED));
    }

    public Command outtake() {
        return Commands.parallel(
                indexer.runIndexer(IntakeConstants.OUTTAKE_SPEED),
                intake.runIntake(IntakeConstants.OUTTAKE_SPEED));
    }

    public Command shootWhenUpToSpeed(double speed) {
        return Commands.parallel(shooter.setShootSpeed(speed),
                Commands.sequence(Commands.waitUntil(() -> shooter.isUpToSpeed(speed)),
                        indexer.runIndexer(IndexerConstants.INDEXER_FEED_SPEED)
                                .withTimeout(IndexerConstants.INDEXER_FEED_TIME)));
    }
}
