package frc.robot.subsystems;


import com.fazecast.jSerialComm.SerialPort;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoIO extends SubsystemBase {
    private SerialPort m_arduinoPort;
    private String m_rgbMessage;
    private Object m_mutex = new Object();

    public ArduinoIO() {
        setRGB(0, 0, 255);
        try {
            
            m_arduinoPort = SerialPort.getCommPort(SerialPort.getCommPorts()[3].getSystemPortPath());
            m_arduinoPort.openPort();
        } catch(Exception e) {
            e.printStackTrace();
        }
    }
    
    @Override
    public void periodic() {
        synchronized(m_mutex) {
            try {
                m_arduinoPort.writeBytes(m_rgbMessage.getBytes(), m_rgbMessage.length());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void setRGB(int red, int green,  int blue) {  
        synchronized(m_mutex) {
            String redString = String.format("%03d", red);
            String greenString = String.format("%03d", green);
            String blueString = String.format("%03d", blue);
            m_rgbMessage = redString + greenString + blueString + ";\n";
        }
    }

   


}


