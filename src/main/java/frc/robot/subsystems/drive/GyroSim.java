package frc.robot.subsystems.drive;

public class GyroSim implements GyroIO {
    private double yaw = 0;

    public double getYaw() {
        return yaw;
    }

    public void setYaw(double yaw) {
        this.yaw = yaw;
    }

    public void updateInputs(GyroIOInputs inputs) {
        return;

    }
}
