package frc.robot.subsystems.drive;

public interface GyroIO {
    public static class GyroIOInputs {
        public double yawPositionRad = 0.0;
    }

    public double getYaw();

    public void setYaw(double yaw);

    public default void updateInputs(GyroIOInputs inputs) {
    }
}
