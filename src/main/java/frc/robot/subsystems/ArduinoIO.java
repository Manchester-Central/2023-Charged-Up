package frc.robot.subsystems;

import java.util.concurrent.ConcurrentHashMap;

import com.fazecast.jSerialComm.SerialPort;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArduinoIO extends SubsystemBase {
    private final int INCOMING_BYTES = 3; // We know beforehand how many bytes we should receive each read cycle. In this, we should only receive one byte for each RGB value.

    private SerialPort m_arduino;
    private RGB m_rgbValues;
    private Object mutexObject;

    public ArduinoIO() {
        m_arduino = SerialPort.getCommPort(Constants.CommConstants.arduinoPort);
        mutexObject = new Object();
    }

    @Override
    public void periodic() {
        m_arduino.openPort();
        if(m_arduino.bytesAvailable() >= 3) {
            byte[] incomingBytes = new byte[INCOMING_BYTES];
            m_arduino.readBytes(incomingBytes, INCOMING_BYTES);
            synchronized(mutexObject) {
                m_rgbValues = new RGB(incomingBytes[0], incomingBytes[1], incomingBytes[2]);
            }
        }
        m_arduino.closePort();
    }

    public RGB getRGBValues() {
        synchronized(mutexObject) {
            return m_rgbValues;
        }
    }


    public class RGB {
        private int R = 0;
        private int G = 0;
        private int B = 0;
        public RGB(int iR, int iG, int iB) {
            R = iR;
            G = iG;
            B = iB;
        }
       
    }
}
