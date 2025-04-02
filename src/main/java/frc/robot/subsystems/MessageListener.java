package frc.robot.subsystems;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import frc.robot.util.AprilTagPIDReading;

public class MessageListener extends SubsystemBase {

    public final int port = Constants.IP_ADDRESS_LISTEN_PORT;
    private final String ipAddress = Constants.LISTEN_IP_ADDRESS;

    // Use volatile for variables accessed by multiple threads
    private volatile String lastMessage = "No messages to display";
    private volatile AprilTagPIDReading aprilTagPIDReading;
    private double timeOfLastMessage = System.currentTimeMillis();


    private DatagramSocket socket;
    private byte[] receiveData;
    private DatagramPacket packet;

    public MessageListener() {
        try {
            // Initialize the socket and buffer
            socket = new DatagramSocket(port, InetAddress.getByName(ipAddress));
            System.out.println("Listening for packets on IP: " + ipAddress + ", Port: " + port);
            receiveData = new byte[512]; // Buffer to hold incoming data
            packet = new DatagramPacket(receiveData, receiveData.length);
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Start the listener thread
        Thread listenerThread = new Thread(() -> {
            try {
                while (true) {
                    // Receive a packet (this call blocks until a packet is received)
                    socket.receive(packet);
                    //System.out.println("Received packet: " + packet.getLength());

                    // Extract the packet data
                    

                    // Synchronize updates to shared resources
                    synchronized (this) {

                        // Update currentDetectedAprilTags
                        aprilTagPIDReading = AprilTagPIDReading.decompress(packet);
                        timeOfLastMessage = System.currentTimeMillis();

                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        });

        listenerThread.setDaemon(true); // Ensure the thread doesn't prevent the program from exiting
        listenerThread.start();
    }

    @Override
    public void periodic() {
        // Perform any periodic updates if necessary
        // Avoid blocking operations and thread creation here
    }

    public String getLastMessage() {
        synchronized (this) {
            return lastMessage;
        }
    }

    public AprilTagPIDReading getAprilTagPIDReading() {
        synchronized (this) {
            return aprilTagPIDReading;
        }
    }


    public double timeSinceLastMessage(){
        return System.currentTimeMillis()-timeOfLastMessage;
    }
    
}