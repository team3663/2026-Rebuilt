package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.*;
import edu.wpi.first.wpilibj.util.Color;
//import com.ctre.phoenix.led.Animation;

public interface LedIo {
    default void updateInputs(LedInputs inputs) {
    }

    default void setColor(Color color) {
    }

    default void setAnimation(ControlRequest animation){
    }

}
