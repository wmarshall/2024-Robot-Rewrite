package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO climberIO;
    private Debouncer leftDebouncer, rightDebouncer = new Debouncer(0.025);

    public static final class ClimberConstants {
        protected static final double CLIMBER_HEIGHT_UPPER_LIMIT = 0.8;
        protected static final double CLIMBER_HEIGHT_LOWER_LIMIT = 0;
        public static final double CLIMBER_SPEED_CONCURRENT = 0.9;
        public static final double CLIMBER_SPEED_SINGLE = 0.5;
    }

    private final double CURRENT_THRESHOLD = 20;

    public ClimberSubsystem(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    
    public Command setConcurrentSpeed(DoubleSupplier speedSupplier) {
        // duplicated implementation because command factories of the same class cannot be parallel-composed
        return this.run(() -> {
            var speed = speedSupplier.getAsDouble();
            if (speed < 0 && !this.isLeftSideStalling() && this.isRightSideStalling()) {
                climberIO.setRightSpeed(0);
            } else {
                climberIO.setRightSpeed(speed);
            }

            if (speed < 0 && !this.isRightSideStalling() && this.isLeftSideStalling()) {
                climberIO.setLeftSpeed(0);
            } else {
                climberIO.setLeftSpeed(speed);
            }
        });
    }

    public Command setRightSpeed(DoubleSupplier speedSupplier) {
        // run() must be the outermost call here so we're not repeatedly running the
        // wrong command if conditions change.
        // Example:
        // At t=0, right side is not stalling, so we construct a runCommand that runs
        // the right motor down
        // At t=1, the right side is stalling (we've hit a hard stop), but we have
        // already constructed and are running a command that will run it down forever
        // For similar reasons, we must use a DoubleSupplier if we want the speed to
        // change over the life of the command
        return this.run(() -> {
            // there is more logic here in the og code for setting the speed to 0 if the
            // climber is outside
            // of the positional range, not needed here because it's directly on the
            // sparkmax
            var speed = speedSupplier.getAsDouble();
            if (speed < 0 && !this.isLeftSideStalling() && this.isRightSideStalling()) {
                climberIO.setRightSpeed(0);
            } else {
                climberIO.setRightSpeed(speed);
            }
            climberIO.setLeftSpeed(0);
        });

    }

    public Command setLeftSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> {
            var speed = speedSupplier.getAsDouble();

            if (speed < 0 && !this.isRightSideStalling() && this.isLeftSideStalling()) {
                climberIO.setLeftSpeed(0);
            } else {
                climberIO.setLeftSpeed(speed);
            }
            climberIO.setRightSpeed(0);

        });
    }

    private boolean isLeftSideStalling() {
        return leftDebouncer.calculate(Math.abs(climberIO.getLeftMotorCurrent()) > CURRENT_THRESHOLD);
    }

    private boolean isRightSideStalling() {
        return rightDebouncer.calculate(Math.abs(climberIO.getRightMotorCurrent()) > CURRENT_THRESHOLD);
    }
}