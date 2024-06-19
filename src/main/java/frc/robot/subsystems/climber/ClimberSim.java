package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSim implements ClimberIO {

    private static final int CANVAS_HEIGHT = 50;
    private static final int CANVAS_WIDTH = 50;
    private static final double MAX_HEIGHT = Units.Meters.convertFrom(20, Units.Inches);
    private static final double MIN_HEIGHT = Units.Meters.convertFrom(0, Units.Inches);
    private static final double START_HEIGHT = Units.Meters.convertFrom(0, Units.Inches);

    private static final double DRUM_RADIUS = Units.Meters.convertFrom(0.5, Units.Inches);
    private static final int REDUCTION = 20;
    private static final double CARRIAGE_MASS = Units.Kilogram.convertFrom(1, Units.Pounds);

    private final ElevatorSim leftElevator = new ElevatorSim(
            DCMotor.getNEO(1),
            REDUCTION,
            CARRIAGE_MASS,
            DRUM_RADIUS,
            MIN_HEIGHT,
            MAX_HEIGHT,
            true,
            MIN_HEIGHT

    );
    private final ElevatorSim rightElevator = new ElevatorSim(
            DCMotor.getNEO(1),
            REDUCTION,
            CARRIAGE_MASS,
            DRUM_RADIUS,
            MIN_HEIGHT,
            MAX_HEIGHT,
            true,
            START_HEIGHT

    );

    private final Mechanism2d elevatorsVisual = new Mechanism2d(CANVAS_WIDTH, CANVAS_HEIGHT);
    private final MechanismRoot2d leftRoot = elevatorsVisual.getRoot("left", 10, 0);
    private final MechanismRoot2d rightRoot = elevatorsVisual.getRoot("right", 40, 0);

    private final MechanismLigament2d leftVisual = leftRoot
            .append(new MechanismLigament2d("Left Extension", leftElevator.getPositionMeters(), 90));

    private final MechanismLigament2d rightVisual = rightRoot
            .append(new MechanismLigament2d("Right Extension", rightElevator.getPositionMeters(), 90));

    public ClimberSim() {
        SmartDashboard.putData("Climber Visualization", elevatorsVisual);
    }

    @Override
    public void setRightSpeed(double speed) {
        if ((speed > 0 && rightElevator.hasHitUpperLimit()) || (speed < 0 && rightElevator.hasHitLowerLimit())) {
            speed = 0;
        }

        rightElevator.setInputVoltage(speed * RobotController.getBatteryVoltage());
    }

    @Override
    public void setLeftSpeed(double speed) {
        if ((speed > 0 && rightElevator.hasHitUpperLimit()) || (speed < 0 && rightElevator.hasHitLowerLimit())) {
            speed = 0;
        }
        leftElevator.setInputVoltage(speed * RobotController.getBatteryVoltage());
    }

    @Override
    public double getRightMotorCurrent() {
        return rightElevator.getCurrentDrawAmps();
    }

    @Override
    public double getLeftMotorCurrent() {
        return leftElevator.getCurrentDrawAmps();
    }

    public void periodic() {
        leftElevator.update(0.02);
        rightElevator.update(0.02);
        leftVisual.setLength(CANVAS_HEIGHT * (leftElevator.getPositionMeters() / (MAX_HEIGHT - MIN_HEIGHT)));
        rightVisual.setLength(CANVAS_HEIGHT * (rightElevator.getPositionMeters() / (MAX_HEIGHT - MIN_HEIGHT)));

    }
}