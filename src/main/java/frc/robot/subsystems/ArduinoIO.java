package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoIO extends SubsystemBase {
    private AddressableLED m_lightStrip;
    private AddressableLEDBuffer m_buffer;
    private Object m_mutex = new Object();

    public ArduinoIO() {
        m_buffer = new AddressableLEDBuffer(30);
        setRGB(200, 200, 200);
        m_lightStrip = new AddressableLED(2);
        m_lightStrip.setLength(m_buffer.getLength());
        m_lightStrip.setData(m_buffer);
        m_lightStrip.start();
    }
    
    @Override
    public void periodic() {
    }

    public void setRGB(int red, int green,  int blue) {  
            for(int i = 0; i < m_buffer.getLength(); i++) {
                m_buffer.setRGB(i, red, green, blue);
            }
    }
}


