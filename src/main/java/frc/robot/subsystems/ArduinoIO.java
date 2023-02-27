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
    private PeripheralData m_PeripheralDataValues = new PeripheralData(0, 0, 0, 0, 0);
    private int proximityData = 0;
    private Object PeripheralDataMutex = new Object();
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
            SmartDashboard.putNumber("Red", (double) m_PeripheralDataValues.R); 
            SmartDashboard.putNumber("Green", (double) m_PeripheralDataValues.G); 
            SmartDashboard.putNumber("Blue", (double) m_PeripheralDataValues.B); 
            SmartDashboard.putNumber("Infared", m_PeripheralDataValues.IR);
            SmartDashboard.putNumber("Proximity Sensor", m_PeripheralDataValues.proximity);
    }

    public PeripheralData getPeripheralDataValues() {
        synchronized(PeripheralDataMutex) {
            return m_PeripheralDataValues;
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
        int proximityData = incomingBytes[0]&0xff; // TODO test these values when we have the robot.
        int infared = (incomingBytes[2]&0xff)|((incomingBytes[3]&0xff)<<8); // TODO test this value when we have the robot.
        int green = (incomingBytes[5]&0xff)|((incomingBytes[6]&0xff)<<8);
        int blue = (incomingBytes[8]&0xff)|((incomingBytes[9]&0xff)<<8);
        int red = (incomingBytes[11]&0xff)|(((incomingBytes[12]&0xff)<<8));
        synchronized(PeripheralDataMutex) {
            m_PeripheralDataValues = new PeripheralData(red, green, blue, infared, proximityData);
        }
    }

    public class PeripheralData {
        private int R = 0;
        private int G = 0;
        private int B = 0;
        private int IR = 0;
        private int proximity;
        public PeripheralData(int iR, int iG, int iB, int iIR, int iProx) {
            R = iR;
            G = iG;
            B = iB;
            IR = iIR;
            proximity = iProx;
        }
       
    }

    // This function will be available to all subsystems and commands. However, it can only be utilized by one at a time.
    public synchronized void queueLEDS(int red, int green, int blue, LED target) {
        m_arduino.writeBytes(new byte[] {(byte) red, (byte) green, (byte) blue, (byte) ((target==LED.CURRENT_GAME_PIECE_INDICATOR) ? 0:1)}, 4); // if the desired LED is CURRENT_GAME_PIECE_INDICATOR,  send 0. If not, send 1.
    }

    public enum LED {
        CURRENT_GAME_PIECE_INDICATOR, DESIRED_GAME_PIECE_INDICATOR
    }
}