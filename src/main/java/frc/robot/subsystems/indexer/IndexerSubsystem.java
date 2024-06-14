package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    private final IndexerIO indexerIO;

    // this is an excellent example of this pattern being bad
    public static final class IndexerConstants {
        public static final double INDEXER_FEED_SPEED = 1.0;
        public static final double INDEXER_FEED_TIME = 1.0;
    }

    public IndexerSubsystem(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

    public Command runIndexer(double speed) {
        return this.run(() -> indexerIO.setMotor(speed));
    }

    public boolean isBeamBroken() {
        return indexerIO.isBeamBroken();
    }
}
