package frc.robot.subsystems.indexer;

public interface IndexerIO {
    public static final class Placebo implements IndexerIO {

        @Override
        public boolean isBeamBroken() {
            return false;
        }

        @Override
        public void setMotor(double speed) {
        }

    }

    public boolean isBeamBroken();

    public void setMotor(double speed);
}