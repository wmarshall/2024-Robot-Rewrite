package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeConstants;

public class CommandFactory {
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;

    public CommandFactory(IndexerSubsystem indexer, IntakeSubsystem intake) {
        this.indexer = indexer;
        this.intake = intake;
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
            intake.runIntake(IntakeConstants.INTAKE_SPEED)
        );
    }

    public Command outtake() {
        return Commands.parallel(
            indexer.runIndexer(IntakeConstants.OUTTAKE_SPEED), 
            intake.runIntake(IntakeConstants.OUTTAKE_SPEED)
        );
    }
}
