package frc.robot.subsystems.shooter;

public interface ShooterIO {
    public static final class Placebo implements ShooterIO {

        @Override
        public void setMotor(double speed) {
        }

        @Override
        public double getEncoderSpeed() {
            return 0;
        }

    }

    public void setMotor(double speed);

    public double getEncoderSpeed();
}
