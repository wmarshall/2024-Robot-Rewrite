package frc.robot.subsystems.climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO climberIO;
    private Debouncer leftDebouncer, rightDebouncer = new Debouncer(0.025);

    public static final double CLIMBER_HEIGHT_UPPER_LIMIT = 0.8;
    public static final double CLIMBER_HEIGHT_LOWER_LIMIT = 0;
    private final double CURRENT_THRESHOLD = 20;

    public ClimberSubsystem(ClimberIO climberIO){
        this.climberIO = climberIO;
    }

    public void setConcurrentSpeed(double speed){
        this.setRightSpeed(speed);
        this.setLeftSpeed(speed);
    }

    public void setRightSpeed(double speed){
        // there is more logic here in the og code for setting the speed to 0 if the climber is outside 
        // of the positional range, not needed here because it's directly on the sparkmax
        if(speed < 0 && !this.isLeftSideStalling() && this.isRightSideStalling()){
            climberIO.setRightSpeed(0);
        }
        else {
            climberIO.setRightSpeed(speed);
        }
    }

    public void setLeftSpeed(double speed){
        if(speed < 0 && !this.isRightSideStalling() && this.isLeftSideStalling()){
            climberIO.setLeftSpeed(0);
        }
        else {
            climberIO.setLeftSpeed(speed);
        }
    }

    public boolean isLeftSideStalling() {
        return leftDebouncer.calculate(Math.abs(climberIO.getLeftMotorCurrent()) > CURRENT_THRESHOLD);
    }

    public boolean isRightSideStalling() {
        return rightDebouncer.calculate(Math.abs(climberIO.getRightMotorCurrent()) > CURRENT_THRESHOLD);
    }
}