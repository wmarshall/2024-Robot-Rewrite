package frc.robot.subsystems.climber;

public interface ClimberIO {
    public void setRightSpeed(double speed);

    public void setLeftSpeed(double speed);

    public double getRightMotorCurrent();

    public double getLeftMotorCurrent();
    public void periodic();
}
