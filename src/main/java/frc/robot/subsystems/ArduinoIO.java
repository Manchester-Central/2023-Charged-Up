package frc.robot.subsystems;

import java.util.concurrent.ConcurrentHashMap;

import com.fazecast.jSerialComm.SerialPort;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// TODO clean up code and prioritize readability.

public class ArduinoIO extends SubsystemBase {
    private final int NUM_BYTES_TO_RECEIVE = 12; // We know beforehand how many bytes we should receive each read cycle. Check color sensor documentation.
    private SerialPort m_arduino;
        private RGB m_rgbValues = new RGB(0, 0, 0);
        private Object mutexObject = new Object();

    public ArduinoIO() {
        m_arduino = SerialPort.getCommPort(Constants.CommConstants.arduinoPort);
    }

    @Override
    public void periodic() {
        m_arduino.openPort();
            if(m_arduino.bytesAvailable() >= 12) {
                readAndInterpretColors();
            }
            SmartDashboard.putNumber("Red", (double) m_rgbValues.R); 
            SmartDashboard.putNumber("Green", (double) m_rgbValues.G); 
            SmartDashboard.putNumber("Blue", (double) m_rgbValues.B); 
    }

    public RGB getRGBValues() {
        synchronized(mutexObject) {
            return m_rgbValues;
        }
    }

    /**
  Index values:
  [0-1]: Proximity sensor.
  [2-4]: Infared.
  [5-7]: Green sensor data.
  [8-10]: Blue sensor data.
  [11-13]: Red sensor data.
*/

    private void readProximityData() {

    }

    private void readInfaredData() {
        
    }

    private void readAndInterpretColors() {
        byte[] incomingBytes = new byte[NUM_BYTES_TO_RECEIVE];
        m_arduino.readBytes(incomingBytes, NUM_BYTES_TO_RECEIVE);
        int green = (incomingBytes[5]&0xff)|((incomingBytes[6]&0xff)<<8);
        int blue = (incomingBytes[8]&0xff)|((incomingBytes[9]&0xff)<<8);
        int red = (incomingBytes[11]&0xff)|(((incomingBytes[12]&0xff)<<8));

        if(red > blue && red > green && red < 210) { // Color correction. These values were aquired via testing.
            red += 75;
        }
        synchronized(mutexObject) {
            m_rgbValues = new RGB(red, green, blue);
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