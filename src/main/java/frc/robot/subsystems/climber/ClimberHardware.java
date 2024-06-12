package frc.robot.subsystems.climber;

import frc.robot.Constants.MotorIdConstants;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberConstants;
import frc.robot.Constants.MotorConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberHardware implements ClimberIO {
    private final CANSparkMax rightMotor, leftMotor;

    public ClimberHardware(){
        rightMotor = new CANSparkMax(MotorIdConstants.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
        leftMotor = new CANSparkMax(MotorIdConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);

        rightMotor.restoreFactoryDefaults();
        leftMotor.restoreFactoryDefaults();

        rightMotor.setSmartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT);
        leftMotor.setSmartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT);

        //probably could make this stuff better, leaving for now
        leftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)(ClimberConstants.CLIMBER_HEIGHT_LOWER_LIMIT + .01));
        leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        rightMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)(ClimberConstants.CLIMBER_HEIGHT_LOWER_LIMIT + .01));
        rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        leftMotor.setSoftLimit(SoftLimitDirection.kForward, (float)(ClimberConstants.CLIMBER_HEIGHT_UPPER_LIMIT - .01));
        leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        rightMotor.setSoftLimit(SoftLimitDirection.kForward, (float)(ClimberConstants.CLIMBER_HEIGHT_UPPER_LIMIT - .01));
        rightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    @Override
    public void setRightSpeed(double speed) {
        rightMotor.set(speed);
    }

    @Override
    public void setLeftSpeed(double speed) {
       leftMotor.set(speed);
    }

    @Override
    public double getRightMotorCurrent() {
        return rightMotor.getOutputCurrent();
    }

    @Override
    public double getLeftMotorCurrent() {
        return leftMotor.getOutputCurrent();
    }
}