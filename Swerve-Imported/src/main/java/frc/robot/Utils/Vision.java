package frc.robot.Utils;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Swerve;

public class Vision {
     private int m_port;
    private DatagramSocket m_socket;
    private DatagramPacket m_packet;
    private Thread m_visionThread;
    private float[] m_locals={0,0,0};
    private float[] m_lastLocals={0,0,0};
    
    public Vision(int port){
        this.m_port = port;
        try{ 
            m_socket = new DatagramSocket(m_port, InetAddress.getByName("0.0.0.0"));
            m_socket.setBroadcast(true);
            byte[] buf = new byte[24];
            m_packet = new DatagramPacket(buf, buf.length);
        }
        catch (Exception e){
            e.printStackTrace();
        }

        m_visionThread = new Thread(()->{
            while(true){
                try {
                    m_socket.receive(m_packet);
                } catch (IOException e) {
                    e.printStackTrace();
                }
                        float[] new_locals = new float[]{(ByteBuffer.wrap(m_packet.getData()).order(ByteOrder.LITTLE_ENDIAN).getFloat()),
                            (ByteBuffer.wrap(m_packet.getData()).order(ByteOrder.LITTLE_ENDIAN).getFloat(4)),
                            (ByteBuffer.wrap(m_packet.getData()).order(ByteOrder.LITTLE_ENDIAN).getFloat(8))};
                        for(int i=0; i<m_locals.length; i++){
                            m_lastLocals[i] = m_locals[i];
                        }
                        for(int i=0; i<m_locals.length; i++){
                            m_locals[i] = new_locals[i];
                        }
                    //put localization values from vision in swerve (not in the correct order because axises are flipped)
                    Swerve.getInstance(Constants.SwerveValues.USES_ABS_ENCODER).setOdometryVals(m_locals[2], m_locals[0]);
            }
        });
        m_visionThread.setDaemon(true);
        m_visionThread.start();
    }

    // public void getLocals(){
        
                
    // }

    public float[] getXYZ(){
        // this.getLocals();
        float[] newLocals={0,0,0};
        for(int i=0; i<m_locals.length; i++){
            newLocals[i] = (m_locals[i] + m_lastLocals[i])/2;
        }
        return m_locals; 
    }

    public float getX(){
        return m_locals[0];
    }
    public float getY(){
        return m_locals[1];
    }
    public float getZ(){
        return m_locals[2];
    }
    

    public double getAngleY(){
        // this.getLocals();
        float[] temp=getXYZ();
        return Math.toDegrees(Math.atan(temp[0]/temp[2]))*-1;
    }

    public double getAngleX(){
        // this.getLocals();
        float[] temp=getXYZ();
        return Math.toDegrees(Math.atan(temp[1]/temp[2]))*-1;
    }    
}
