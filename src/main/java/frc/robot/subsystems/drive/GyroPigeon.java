package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SensorConstants;

public class GyroPigeon implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroPigeon() {

        pigeon = new Pigeon2(SensorConstants.GYRO_ID, "rio");
        this.setYaw(0.0);

    }

    public double getYaw() {
        return Units.degreesToRadians(pigeon.getYaw().getValue());
    }

    public void setYaw(double yaw) {
        pigeon.setYaw(Units.radiansToDegrees(yaw));
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawPositionRad = pigeon.getYaw().getValue();

    }
}
