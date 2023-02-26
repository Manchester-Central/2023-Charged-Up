package frc.robot.subsystems;

import java.util.concurrent.ConcurrentHashMap;

import com.fazecast.jSerialComm.SerialPort;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// TODO clean up code and prioritize readability.

public class ArduinoIO extends SubsystemBase {
    private final int NUM_BYTES_TO_RECEIVE = 14; // We know beforehand how many bytes we should receive each read cycle. Check color sensor documentation.
    private SerialPort m_arduino;
    private RGBIR m_RGBIRirValues = new RGBIR(0, 0, 0, 0);
    private int proximityData = 0;
    private Object rgbirMutex = new Object();
    private Object infaredMutex = new Object();

    public ArduinoIO() {
        m_arduino = SerialPort.getCommPort(Constants.CommConstants.arduinoPort);
    }

    @Override
    public void periodic() {
        m_arduino.openPort();
            if(m_arduino.bytesAvailable() >= NUM_BYTES_TO_RECEIVE) {
                readArduinoOutput();
            }
            SmartDashboard.putNumber("Red", (double) m_RGBIRirValues.R); 
            SmartDashboard.putNumber("Green", (double) m_RGBIRirValues.G); 
            SmartDashboard.putNumber("Blue", (double) m_RGBIRirValues.B); 
    }

    public RGBIR getRGBIRValues() {
        synchronized(rgbirMutex) {
            return m_RGBIRirValues;
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

    private void readArduinoOutput() {
        byte[] incomingBytes = new byte[NUM_BYTES_TO_RECEIVE];
        m_arduino.readBytes(incomingBytes, NUM_BYTES_TO_RECEIVE);
        int infared = 0; // TODO test these values when we have the robot.
        proximityData = 0; // TODO test this value when we have the robot.
        int green = (incomingBytes[5]&0xff)|((incomingBytes[6]&0xff)<<8);
        int blue = (incomingBytes[8]&0xff)|((incomingBytes[9]&0xff)<<8);
        int red = (incomingBytes[11]&0xff)|(((incomingBytes[12]&0xff)<<8));
        if(red > blue && red > green && red < 210) { // Color correction. These values were aquired via testing.
            red += 75;
        }
        synchronized(rgbirMutex) {
            m_RGBIRirValues = new RGBIR(red, green, blue, infared);
        }
    }

    public class RGBIR {
        private int R = 0;
        private int G = 0;
        private int B = 0;
        private int IR = 0;
        public RGBIR(int iR, int iG, int iB, int iIR) {
            R = iR;
            G = iG;
            B = iB;
            IR = iIR;
        }
       
    }

    // This function will be available to all subsystems and commands. However, it can only be utilized by one at a time.
    public synchronized void queueLEDS(int red, int green, int blue) {
        m_arduino.writeBytes(new byte[] {(byte) red, (byte) green, (byte) blue}, 3);
    }
}