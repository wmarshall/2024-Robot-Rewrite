package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SensorConstants;

public class IndexerHardware implements IndexerIO {
    private final CANSparkMax indexerMotor;
    private final DigitalInput indexerSensor;

    public IndexerHardware() {
        indexerMotor = new CANSparkMax(MotorIdConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

        indexerMotor.restoreFactoryDefaults();
        indexerMotor.setInverted(false);
        indexerMotor.setSmartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT);

        indexerSensor = new DigitalInput(SensorConstants.BEAM_BREAK_SENSOR_ID);
    }

    @Override
    public boolean isBeamBroken() {
        return !indexerSensor.get();
    }

    @Override
    public void setMotor(double speed) {
        indexerMotor.set(speed);
    }

}
