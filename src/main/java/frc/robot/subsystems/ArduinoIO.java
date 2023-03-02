package frc.robot.subsystems;

import java.io.IOException;
import java.util.concurrent.ConcurrentHashMap;

import com.fazecast.jSerialComm.SerialPort;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// TODO clean up code and prioritize readability.

public class ArduinoIO extends SubsystemBase {
    private final int NUM_BYTES_TO_RECEIVE = 14; // We know beforehand how many bytes we should receive each read cycle. Check color sensor documentation.
    private SerialPort m_arduino;
    private Object PeripheralDataMutex = new Object();
    private byte[] m_peripheralDataValues = new byte[14];

    public ArduinoIO() {
        SerialPort[] ports = SerialPort.getCommPorts();
        for(int i = 0; i < ports.length; i++) {
            SmartDashboard.putString("Serial Port" + Integer.toString(i), ports[i].getDescriptivePortName());
        }
    }

    @Override
    public void periodic() {
        m_arduino.openPort();  
        String data = "";
            if(m_arduino.bytesAvailable() >= NUM_BYTES_TO_RECEIVE) {
                readArduinoOutput();
                for(int i = 0; i < 14; i++) {
                    data += Byte.toString(m_peripheralDataValues[i]) + ",";
                }
                SmartDashboard.putString("Arduino Data", data);
            }
    }

    private void readArduinoOutput() {
        byte[] incomingBytes = new byte[NUM_BYTES_TO_RECEIVE];
        m_arduino.readBytes(incomingBytes, NUM_BYTES_TO_RECEIVE);
        synchronized(PeripheralDataMutex) {
            for(int i = 0; i < NUM_BYTES_TO_RECEIVE; i++) {
                m_peripheralDataValues[i] = incomingBytes[i];
            }
        }
    }


}