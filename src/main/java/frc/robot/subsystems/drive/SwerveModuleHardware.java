package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.drive.SwerveModule.SwerveModuleConstants;

public class SwerveModuleHardware implements SwerveModuleIO {
    private final CANSparkMax drivingMotor;
    private final CANSparkMax steeringMotor;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder steeringEncoder;

    private final SparkPIDController drivingPIDController;
    private final SparkPIDController steeringPIDController;

    private double chassisAngularOffset;
    private String name;

    private static final boolean STEER_ENCODER_INVERTED = true;
    private static final boolean DRIVE_ENCODER_INVERTED = false;

    private static final double STEERING_ENCORDER_POSITION_FACTOR = (2 * Math.PI); // radians
    private static final double STEERING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

    private static final double DRIVING_MOTOR_FREE_SPEED_RPS = MotorConstants.NEO_FREE_SPEED_RPM / 60;
    private static final double WHEEL_DIAMETER_IN_METERS = Units.inchesToMeters(3.0);
    private static final double WHEEL_CIRCUMFERENCE_IN_METERS = WHEEL_DIAMETER_IN_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    // This is also the gear ratio (14T)

    private static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
            * WHEEL_CIRCUMFERENCE_IN_METERS)
            / SwerveModuleConstants.DRIVING_MOTOR_REDUCTION;

    private static final double STEERING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
    private static final double STEERING_ENCODER_POSITION_PID_MAX_INPUT = STEERING_ENCORDER_POSITION_FACTOR; // radians

    private static final double DRIVING_P = 0.2;
    private static final double DRIVING_I = 0;
    private static final double DRIVING_D = 0;
    private static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
    private static final double DRIVING_MIN_OUTPUT = -1;
    private static final double DRIVING_MAX_OUTPUT = 1;

    private static final double STEERING_P = 1.0;
    private static final double STEERING_I = 0;
    private static final double STEERING_D = 0.001;
    private static final double STEERING_FF = 0;
    private static final double STEERING_MIN_OUTPUT = -1;
    private static final double STEERING_MAX_OUTPUT = 1;

    public SwerveModuleHardware(int drivingCANId, int turningCANId, double chassisAngularOffset,
            String name) {

        int errors = 0;

        this.name = name;

        drivingMotor = new CANSparkMax(drivingCANId, MotorType.kBrushless);
        drivingMotor.restoreFactoryDefaults();
        drivingMotor.setSmartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT);
        drivingMotor.setInverted(DRIVE_ENCODER_INVERTED);

        steeringMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);
        steeringMotor.restoreFactoryDefaults();
        steeringMotor.setSmartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT);
        steeringMotor.setInverted(STEER_ENCODER_INVERTED);

        drivingMotor.enableVoltageCompensation(12);
        steeringMotor.enableVoltageCompensation(12);

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        drivingEncoder = drivingMotor.getEncoder();
        steeringEncoder = steeringMotor.getAbsoluteEncoder(Type.kDutyCycle);
        drivingPIDController = drivingMotor.getPIDController();
        steeringPIDController = steeringMotor.getPIDController();
        steeringPIDController.setFeedbackDevice(steeringEncoder);

        errors += check(drivingPIDController.setFeedbackDevice(drivingEncoder));
        errors += check(steeringPIDController.setFeedbackDevice(steeringEncoder));

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        drivingEncoder.setPositionConversionFactor(0.04955);
        drivingEncoder.setVelocityConversionFactor(0.04955 / 60);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        steeringEncoder.setPositionConversionFactor(STEERING_ENCORDER_POSITION_FACTOR);
        steeringEncoder.setVelocityConversionFactor(STEERING_ENCODER_VELOCITY_FACTOR);

        errors += check(
                steeringEncoder.setPositionConversionFactor(STEERING_ENCORDER_POSITION_FACTOR));
        errors += check(
                steeringEncoder.setVelocityConversionFactor(STEERING_ENCODER_VELOCITY_FACTOR));

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        steeringEncoder.setInverted(STEER_ENCODER_INVERTED);
        drivingMotor.setInverted(DRIVE_ENCODER_INVERTED);

        errors += check(steeringEncoder.setInverted(STEER_ENCODER_INVERTED));

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        steeringPIDController.setPositionPIDWrappingEnabled(true);
        steeringPIDController.setPositionPIDWrappingMinInput(STEERING_ENCODER_POSITION_PID_MIN_INPUT);
        steeringPIDController.setPositionPIDWrappingMaxInput(STEERING_ENCODER_POSITION_PID_MAX_INPUT);

        errors += check(steeringPIDController.setPositionPIDWrappingEnabled(true));
        errors += check(steeringPIDController
                .setPositionPIDWrappingMinInput(STEERING_ENCODER_POSITION_PID_MIN_INPUT));
        errors += check(steeringPIDController
                .setPositionPIDWrappingMaxInput(STEERING_ENCODER_POSITION_PID_MAX_INPUT));

        // Set the PID gains for the driving motor
        drivingPIDController.setP(DRIVING_P);
        drivingPIDController.setI(DRIVING_I);
        drivingPIDController.setD(DRIVING_D);
        drivingPIDController.setFF(DRIVING_FF);
        drivingPIDController.setOutputRange(DRIVING_MIN_OUTPUT,
                DRIVING_MAX_OUTPUT);

        errors += check(drivingPIDController.setP(DRIVING_P));
        errors += check(drivingPIDController.setFF(DRIVING_FF));
        errors += check(drivingPIDController.setOutputRange(DRIVING_MIN_OUTPUT,
                DRIVING_MAX_OUTPUT));

        // Set the PID gains for the turning motor
        steeringPIDController.setP(STEERING_P);
        steeringPIDController.setI(STEERING_I);
        steeringPIDController.setD(STEERING_D);
        steeringPIDController.setFF(STEERING_FF);
        steeringPIDController.setOutputRange(STEERING_MIN_OUTPUT,
                STEERING_MAX_OUTPUT);

        errors += check(steeringPIDController.setP(STEERING_P));
        errors += check(steeringPIDController.setFF(STEERING_FF));
        errors += check(steeringPIDController.setOutputRange(STEERING_MIN_OUTPUT,
                STEERING_MAX_OUTPUT));

        this.chassisAngularOffset = chassisAngularOffset;
        drivingEncoder.setPosition(0);

        errors += check(drivingEncoder.setPosition(0));

        if (errors > 0) {
            System.out.println("Swerve Module Errors! Name: " + name + ", Amount: " + errors);
        }
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.driveBusVoltage = getDriveBusVoltage();
        inputs.driveAppliedVolts = getDriveBusVoltage() * getDriveOutput();
        inputs.drivePositionMeters = getDriveEncoderPosition();
        inputs.driveVelocityMPS = getDriveEncoderSpeedMPS();
        inputs.turnBusVoltage = getTurnBusVoltage();
        inputs.turnAppliedVolts = getTurnBusVoltage() * getTurnOutput();
        inputs.turnPositionRad = getTurnEncoderPosition();

    }

    public void setDriveEncoderPosition(double position) {
        drivingEncoder.setPosition(position);
    };

    public double getDriveEncoderPosition() {
        return drivingEncoder.getPosition();
    };

    public void setDesiredDriveSpeedMPS(double speed) {
        drivingPIDController.setReference(speed, ControlType.kVelocity);
    };

    public double getDriveEncoderSpeedMPS() {
        return drivingEncoder.getVelocity();
    };

    public double getTurnEncoderPosition() {
        return steeringEncoder.getPosition();
    };

    public void setDesiredTurnAngle(double angle) {
        steeringPIDController.setReference(angle, ControlType.kPosition);
    };

    public double getDriveBusVoltage() {
        return drivingMotor.getBusVoltage();
    }

    public double getDriveOutput() {
        return drivingMotor.getAppliedOutput();
    }

    public double getTurnBusVoltage() {
        return steeringMotor.getBusVoltage();
    }

    public double getTurnOutput() {
        return steeringMotor.getAppliedOutput();
    }

    public String getName() {
        return name;
    }

    public double getChassisAngularOffset() {
        return chassisAngularOffset;
    }

    private int check(REVLibError err) {
        return err == REVLibError.kOk ? 0 : 1;
    }
}
