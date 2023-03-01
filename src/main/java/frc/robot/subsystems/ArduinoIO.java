package frc.robot.subsystems;

import java.util.concurrent.ConcurrentHashMap;

import com.fazecast.jSerialComm.SerialPort;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// TODO clean up code and prioritize readability.

public class ArduinoIO extends SubsystemBase {
    private final int NUM_BYTES_TO_RECEIVE = 2; // We know beforehand how many bytes we should receive each read cycle. Check color sensor documentation.
    private SerialPort m_arduino;

    public ArduinoIO() {
        m_arduino = SerialPort.getCommPort(Constants.CommConstants.arduinoPort);
    }

    @Override
    public void periodic() {
        m_arduino.openPort();
            if(m_arduino.bytesAvailable() >= NUM_BYTES_TO_RECEIVE) {
                readArduinoOutput();
            }
    }

    private void readArduinoOutput() {
    }


    // This function will be available to all subsystems and commands. However, it can only be utilized by one at a time.
    public synchronized void queueLEDS(int red, int green, int blue) {
        m_arduino.writeBytes(new byte[] {(byte) red, (byte) green, (byte) blue}, 3);
    }
}