package frc.robot.subsystems.climber;

public interface ClimberIO {
    public static final class Placebo implements ClimberIO {

        @Override
        public void setRightSpeed(double speed) {
        }

        @Override
        public void setLeftSpeed(double speed) {
        }

        @Override
        public double getRightMotorCurrent() {
            return 0;
        }

        @Override
        public double getLeftMotorCurrent() {
            return 0;
        }

    }

    public void setRightSpeed(double speed);

    public void setLeftSpeed(double speed);

    public double getRightMotorCurrent();

    public double getLeftMotorCurrent();
}
