package frc.robot.subsystems;

import com.fazecast.jSerialComm.SerialPort;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CommConstants;

public class ArduinoIO extends SubsystemBase {
    private SerialPort m_arduinoPort;
    public byte[] m_rgbValues = {0, 0, 0};
    private Object m_mutex = new Object();

    public ArduinoIO() {
        try {
            m_arduinoPort = SerialPort.getCommPort(CommConstants.arduinoPort); 
        } catch(Exception e) {
            e.printStackTrace();
        }
    }
    
    @Override
    public void periodic() {
        synchronized(m_mutex) {
        try {
            m_arduinoPort.writeBytes(m_rgbValues, 3);
        } catch (Exception e) {
            e.printStackTrace();
        }
        }
    }

    public void setRGB(byte R, byte G, byte B) {
        synchronized(m_mutex) {
            m_rgbValues[0] = R;
            m_rgbValues[1] = G;
            m_rgbValues[2] = B;
        }
    }

   


}


