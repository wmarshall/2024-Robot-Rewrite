package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterConstants;

public class ShooterHardware implements ShooterIO {
    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;
    public static RelativeEncoder shooterTopEncoder;
    public static RelativeEncoder shooterBottomEncoder;
    public static SparkPIDController shooterTopController;
    public static SparkPIDController shooterBottomController;

    public ShooterHardware() {
        topMotor = new CANSparkMax(MotorIdConstants.SHOOTER_TOP_MOTOR_ID, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(MotorIdConstants.SHOOTER_BOTTOM_MOTOR_ID, MotorType.kBrushless);

        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        topMotor.setInverted(true);
        bottomMotor.setInverted(true);

        topMotor.setSmartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT);
        bottomMotor.setSmartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT);

        shooterTopEncoder = topMotor.getEncoder();
        shooterBottomEncoder = bottomMotor.getEncoder();

        shooterTopEncoder.setVelocityConversionFactor(MotorConstants.MINUTE_TO_SECOND_CONVERSION);
        shooterBottomEncoder.setVelocityConversionFactor(MotorConstants.MINUTE_TO_SECOND_CONVERSION);

        shooterTopController = topMotor.getPIDController();
        shooterBottomController = bottomMotor.getPIDController();

        shooterTopController.setFeedbackDevice(shooterTopEncoder);
        shooterBottomController.setFeedbackDevice(shooterBottomEncoder);

        shooterTopController.setOutputRange(0, 1);
        shooterBottomController.setOutputRange(0, 1);

        shooterTopController.setFF(ShooterConstants.SHOOTER_FEEDFORWARD);
        shooterBottomController.setFF(ShooterConstants.SHOOTER_FEEDFORWARD);

        shooterTopController.setP(ShooterConstants.SHOOTER_PVALUE);
        shooterBottomController.setP(ShooterConstants.SHOOTER_PVALUE);

    }

    @Override
    public void setMotor(double speed) {
        topMotor.set(speed);
        bottomMotor.set(speed);
    }

    @Override
    public double getEncoderSpeed() {
        return (shooterTopEncoder.getVelocity() + shooterBottomEncoder.getVelocity()) / 2;
    }

}
