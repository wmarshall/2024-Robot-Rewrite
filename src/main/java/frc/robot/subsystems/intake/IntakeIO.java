package frc.robot.subsystems.intake;

public interface IntakeIO {
    public static final class Placebo implements IntakeIO {

        @Override
        public void setMotors(double speed) {
        }

    }

    public void setMotors(double speed);
}
