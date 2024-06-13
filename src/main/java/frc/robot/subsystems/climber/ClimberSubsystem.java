package frc.robot.subsystems.climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO climberIO;
    private Debouncer leftDebouncer, rightDebouncer = new Debouncer(0.025);

    public static final class ClimberConstants {
        public static final double CLIMBER_HEIGHT_UPPER_LIMIT = 0.8;
        public static final double CLIMBER_HEIGHT_LOWER_LIMIT = 0;
        public static final double CLIMBER_SPEED_CONCURRENT = 0.9;
        public static final double CLIMBER_SPEED_SINGLE = 0.5;
    }
    
    private final double CURRENT_THRESHOLD = 20;

    public ClimberSubsystem(ClimberIO climberIO){
        this.climberIO = climberIO;
    }

    //I have a bad feeling this isn't going to work due to requirement issues, don't really want to deal with it
    public Command setConcurrentSpeed(double speed){
        return Commands.parallel(this.setRightSpeed(speed), this.setLeftSpeed(speed));
    }

    public Command setRightSpeed(double speed){
        // there is more logic here in the og code for setting the speed to 0 if the climber is outside 
        // of the positional range, not needed here because it's directly on the sparkmax
        if(speed < 0 && !this.isLeftSideStalling() && this.isRightSideStalling()){
            return this.run(() -> climberIO.setRightSpeed(0));
        }
        else {
            return this.run(() -> climberIO.setRightSpeed(speed));
        }
    }

    public Command setLeftSpeed(double speed){
        if(speed < 0 && !this.isRightSideStalling() && this.isLeftSideStalling()) {
            return this.run(() -> climberIO.setLeftSpeed(0));
        }
        else {
            return this.run(() -> climberIO.setLeftSpeed(speed));
        }
    }

    private boolean isLeftSideStalling() {
        return leftDebouncer.calculate(Math.abs(climberIO.getLeftMotorCurrent()) > CURRENT_THRESHOLD);
    }

    private boolean isRightSideStalling() {
        return rightDebouncer.calculate(Math.abs(climberIO.getRightMotorCurrent()) > CURRENT_THRESHOLD);
    }
}