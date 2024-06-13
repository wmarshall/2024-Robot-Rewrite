package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public static final class IntakeConstants {
        protected static final double INTAKE_FEEDFORWARD = 0.01;
        protected static final double INTAKE_PVALUE = 0.01;
        public static final double OUTTAKE_SPEED = -0.3;
        public static final double INTAKE_SPEED = 1;
    }
    private final IntakeIO intakeIO;

    public IntakeSubsystem(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    public Command RunIntake(double speed) {
        return this.run(() -> intakeIO.setMotors(speed));
    }
}
