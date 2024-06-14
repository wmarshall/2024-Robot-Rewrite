package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public static final class ShooterConstants {
        protected static double SHOOTER_FEEDFORWARD = 0.011;
        protected static double SHOOTER_PVALUE = 0.01;
        protected static double SHOOTER_MAX_RPS = 94.0;
        public static double SHOOTER_SPEED = 1.0;
    }

    private final ShooterIO shooterIO;

    public ShooterSubsystem(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }

    public Command setShootSpeed(double speed) {
        return this.run(() -> shooterIO.setMotor(speed));
    }

    public boolean isUpToSpeed(double speed) {
        return shooterIO.getEncoderSpeed() >= (speed * ShooterConstants.SHOOTER_MAX_RPS);
    }
}
