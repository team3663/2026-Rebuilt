package frc.robot.subsystems.hopper;

import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    private final HopperIO io;
    private final Constants constants;
    private final HopperInputsAutoLogged inputs = new HopperInputsAutoLogged();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    private final DoubleCircularBuffer rollerCurrentDrawBuffer = new DoubleCircularBuffer(35);

    public Hopper(HopperIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }


    public enum WantedState {
        FEEDING,
        EJECTING,
        IDLE
    }

    public enum SystemState {
        FEEDING,
        EJECTING,
        IDLE
    }

    public void periodic() {
        io.updateInputs(inputs);

        systemState = handleStateTransition();

        rollerCurrentDrawBuffer.addFirst(inputs.topRollerCurrentDraw);

        Logger.processInputs("Hopper/Inputs", inputs);
        Logger.recordOutput("Hopper/WantedState", wantedState);
        Logger.recordOutput("Hopper/SystemState", systemState);
        Logger.recordOutput("Hopper/AverageRollerCurrentDraw", getAverageRollerCurrentDraw());

        applyState();
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case FEEDING:
                yield SystemState.FEEDING;
            case EJECTING:
                yield SystemState.EJECTING;
            default:
                yield SystemState.IDLE;
        };
    }

    private void applyState() {
        double hopperVoltage = 0.0;
        double tunnelVoltage = 0.0;
        double topRollerVoltage = 0.0;

        switch (systemState) {
            case FEEDING:
                hopperVoltage = constants.hopperVoltage;
                tunnelVoltage = constants.tunnelVoltage;
                topRollerVoltage = constants.rollerVoltage;
                break;
            case EJECTING:
                hopperVoltage = constants.hopperVoltage;
                tunnelVoltage = constants.tunnelVoltage;
                topRollerVoltage = constants.rollerEjectingVoltage;
                break;
            case IDLE:
                hopperVoltage = 0.0;
                tunnelVoltage = 0.0;
                topRollerVoltage = 0.0;
                break;
        }

        io.setTargetVoltage(tunnelVoltage, hopperVoltage, topRollerVoltage);
    }


    public double getAverageRollerCurrentDraw() {
        double sumOfNumbers = 0;

        for (int i = 0; i < rollerCurrentDrawBuffer.size(); i++) {
            sumOfNumbers += rollerCurrentDrawBuffer.get(i);
        }
        return sumOfNumbers / rollerCurrentDrawBuffer.size();
    }

    public void clearTopRollerAverageCurrentDraw() {
        rollerCurrentDrawBuffer.clear();
    }

    /**
     * @param hopperVoltage         - Voltage to run the hopper at
     * @param tunnelVoltage         - Voltage to run the tunnel motors at
     * @param rollerVoltage         - Voltage to run the top roller at
     * @param rollerEjectingVoltage - Voltage to run the top roller when ejecting
     */
    public record Constants(double hopperVoltage, double tunnelVoltage, double rollerVoltage,
                            double rollerEjectingVoltage) {
    }
}