package frc.robot.subsystems;

import java.io.IOException;
import java.util.concurrent.ConcurrentHashMap;

import com.fazecast.jSerialComm.SerialPort;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// TODO clean up code and prioritize readability.

public class ArduinoIO extends SubsystemBase {
    private final int NUM_BYTES_TO_RECEIVE = 2; // We know beforehand how many bytes we should receive each read cycle. Check color sensor documentation.
    private SerialPort m_arduino;
    private PeripheralData m_PeripheralDataValues = new PeripheralData(0, 0, 0);
    private Object PeripheralDataMutex = new Object();

    public ArduinoIO() {
        m_arduino = SerialPort.getCommPort("/dev/ttyS0");
        SmartDashboard.putString("Arduino test", m_arduino.getDescriptivePortName());
        m_arduino.openPort();
    }

    @Override
    public void periodic() {
       byte[] in = {0};
            SmartDashboard.putNumber("num Bytes", m_arduino.writeBytes(new byte[] {(byte) 50}, 1));
            /*if(m_arduino.bytesAvailable() >= 1) {
                m_arduino.readBytes(in ,1);
            }*/
            SmartDashboard.putNumber("ReadData", (double) in[0]);
    }

    public PeripheralData getPeripheralDataValues() {
        synchronized(PeripheralDataMutex) {
            return m_PeripheralDataValues;
        }
    }

    private void readArduinoOutput() {
        byte[] incomingBytes = new byte[NUM_BYTES_TO_RECEIVE];
        m_arduino.readBytes(incomingBytes, NUM_BYTES_TO_RECEIVE);
        synchronized(PeripheralDataMutex) {
            m_PeripheralDataValues = new PeripheralData(incomingBytes[0], incomingBytes[1], incomingBytes[2]);
        }
    }

    public class PeripheralData {
        private int hue;
        private int IR = 0;
        private int proximity;
        public PeripheralData(int iHue, int iIR, int iProx) {
            hue = iHue;
            IR = iIR;
            proximity = iProx;
        }
    }

    // This function will be available to all subsystems and commands. However, it can only be utilized by one at a time.
    public synchronized void queueLEDS(int red, int green, int blue) {
        m_arduino.writeBytes(new byte[] {(byte) red, (byte) green, (byte) blue}, 3); // if the desired LED is CURRENT_GAME_PIECE_INDICATOR,  send 0. If not, send 1.
    }
}