package frc.robot.subsystems;


import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoIO extends SubsystemBase {
    private SerialPort m_arduinoPort;
    private String m_rgbMessage;
    private Object m_mutex = new Object();

    public ArduinoIO() {
        setRGB(255, 255, 255);
        try {
            m_arduinoPort = new SerialPort(9600, SerialPort.Port.kUSB1);
        } catch(Exception e) {
            e.printStackTrace();
        }
    }
    
    @Override
    public void periodic() {
        synchronized(m_mutex) {
            try {
                m_arduinoPort.writeString(m_rgbMessage);
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
            m_rgbMessage = redString + greenString + blueString + ";";
        }
    }

   


}


