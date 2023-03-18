package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LedStrip extends SubsystemBase {
    private AddressableLED  m_ledStrip;
    private Consumer<AddressableLED> ledState;
    
    public static enum Strip {
        TOP, GRIPPER
    }

    public LedStrip(int port) {
        m_ledStrip = new AddressableLED(port);
    }

    @Override
    public void periodic() {
        
    }


    public void setLedState(Consumer<AddressableLED> state) {
        ledState = state;
    }
}
