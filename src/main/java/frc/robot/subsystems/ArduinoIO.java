package frc.robot.subsystems;


import com.fazecast.jSerialComm.SerialPort;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoIO extends SubsystemBase {
    private SerialPort m_arduinoPort;
    private String m_rgbMessage;
    private Object m_mutex = new Object();

    public ArduinoIO() {
        setRGB(255, 255, 255);
        try {
            SerialPort[] ports = SerialPort.getCommPorts();
            for(int i = 0; i < ports.length; i++) {
              //  SmartDashboard.putString("Port path " + Integer.toString(i), ports[i].getSystemPortPath());
              //  SmartDashboard.putString("Port name " + Integer.toString(i), ports[i].getDescriptivePortName());
            }
            m_arduinoPort = ports[3];
            m_arduinoPort.openPort();
            
        } catch(Exception e) {
            e.printStackTrace();
        }
    }
    
    @Override
    public void periodic() {
        synchronized(m_mutex) {
            try {
                    SmartDashboard.putBoolean("isPortOpen", m_arduinoPort.isOpen());
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
            SmartDashboard.putString("rgb", m_rgbMessage);
        }
    }

   


}


