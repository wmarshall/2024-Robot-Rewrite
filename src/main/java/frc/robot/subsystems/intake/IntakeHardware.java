package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeConstants;

public class IntakeHardware implements IntakeIO {
    private CANSparkMax leftCenteringIntakeMotor;
    private CANSparkMax rightCenteringIntakeMotor;
    private CANSparkMax intakeMotor;
    private RelativeEncoder intakeEncoder;
    private SparkPIDController intakePIDController;

    public IntakeHardware() {
        leftCenteringIntakeMotor = new CANSparkMax(MotorIdConstants.LEFT_INTAKE_CENTERING_MOTOR_ID, MotorType.kBrushless);
        rightCenteringIntakeMotor = new CANSparkMax(MotorIdConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(MotorIdConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

        leftCenteringIntakeMotor.setSmartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT);
        rightCenteringIntakeMotor.setSmartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT);
        intakeMotor.setSmartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT);

        leftCenteringIntakeMotor.restoreFactoryDefaults();
        rightCenteringIntakeMotor.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();

        leftCenteringIntakeMotor.setInverted(true);
        rightCenteringIntakeMotor.setInverted(false);
        intakeMotor.setInverted(false);

        intakeEncoder = intakeMotor.getEncoder();
        intakeEncoder.setVelocityConversionFactor(1/60.0);

        intakePIDController = intakeMotor.getPIDController();
        intakePIDController.setFeedbackDevice(intakeEncoder);
        
        intakePIDController.setFF(IntakeConstants.INTAKE_FEEDFORWARD);
        intakePIDController.setP(IntakeConstants.INTAKE_PVALUE);
    }

    @Override
    public void setMotors(double speed) {
        leftCenteringIntakeMotor.set(speed);
        rightCenteringIntakeMotor.set(speed);
        intakeMotor.set(speed);
    }
}
