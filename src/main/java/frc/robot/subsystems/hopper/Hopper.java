package frc.robot.subsystems.hopper;

import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.Feeder;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    private final HopperIO io;
    private final Constants constants;
    private final HopperInputsAutoLogged inputs = new HopperInputsAutoLogged();

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;

//    private final DoubleCircularBuffer rollerCurrentDrawBuffer = new DoubleCircularBuffer(35);

    public Hopper(HopperIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }


    public enum WantedState {
        FEEDING,
        EJECTING,
        DEFAULT,
        OFF
    }

    public enum SystemState {
        FEEDING,
        EJECTING,
        DEFAULT,
        OFF
    }

    public void periodic() {
        io.updateInputs(inputs);
//        rollerCurrentDrawBuffer.addFirst(inputs.topRollerCurrentDraw);
        Logger.processInputs("Hopper/Inputs", inputs);
        Logger.recordOutput("Hopper/WantedState", wantedState);
        Logger.recordOutput("Hopper/SystemState", systemState);
//        Logger.recordOutput("Hopper/AverageRollerCurrentDraw", getAverageRollerCurrentDraw());
    }

    private SystemState handleStateTransition(){
        return switch (wantedState) {
            case FEEDING:
                yield SystemState.FEEDING;
            case EJECTING:
                yield SystemState.EJECTING;
            default:
                yield SystemState.OFF;
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
            case EJECTING:
                hopperVoltage = constants.hopperVoltage;
                tunnelVoltage = constants.tunnelVoltage;
                topRollerVoltage = constants.rollerEjectingVoltage;
            case OFF:
            case DEFAULT:
                hopperVoltage = 0.0;
                tunnelVoltage = 0.0;
                topRollerVoltage = 0.0;
        }

        io.setTargetVoltage(tunnelVoltage, hopperVoltage, topRollerVoltage);
    }


//    public double getAverageRollerCurrentDraw() {
//        double sumOfNumbers = 0;
//
//        for (int i = 0; i < rollerCurrentDrawBuffer.size(); i++) {
//            sumOfNumbers += rollerCurrentDrawBuffer.get(i);
//        }
//        return sumOfNumbers / rollerCurrentDrawBuffer.size();
//    }
//
//    public void clearTopRollerAverageCurrentDraw() {
//        rollerCurrentDrawBuffer.clear();
//    }

    /**
     * @param hopperVoltage - Voltage to run the hopper at
     * @param tunnelVoltage - Voltage to run the tunnel motors at
     * @param rollerVoltage - Voltage to run the top roller at
     * @param rollerEjectingVoltage - Voltage to run the top roller when ejecting
     */
    public record Constants(double hopperVoltage, double tunnelVoltage, double rollerVoltage, double rollerEjectingVoltage) {
    }
}