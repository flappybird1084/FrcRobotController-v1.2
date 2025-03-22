package frc.robot.deprecated;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.AprilTagDetected;

public class AprilTagDetectedMessageListener extends SubsystemBase {

    public final int port = Constants.IP_ADDRESS_LISTEN_PORT;
    private final String ipAddress = Constants.LISTEN_IP_ADDRESS;

    // Use volatile for variables accessed by multiple threads
    private volatile String lastMessage = "No messages to display";
    private volatile ArrayList<AprilTagDetected> currentDetectedAprilTags = new ArrayList<>();

    private DatagramSocket socket;
    private byte[] receiveData;
    private DatagramPacket packet;

    public AprilTagDetectedMessageListener() {
        try {
            // Initialize the socket and buffer
            socket = new DatagramSocket(port, InetAddress.getByName(ipAddress));
            System.out.println("Listening for packets on IP: " + ipAddress + ", Port: " + port);
            receiveData = new byte[16384]; // Buffer to hold incoming data
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

                    // Extract the packet data
                    String receivedMessage = new String(packet.getData(), 0, packet.getLength());

                    // Synchronize updates to shared resources
                    synchronized (this) {
                        lastMessage = receivedMessage;

                        // Update currentDetectedAprilTags
                        String[] split_strings = lastMessage.split("-next-detection-");
                        ArrayList<AprilTagDetected> tags = new ArrayList<>();
                        for (String tag : split_strings) {
                            try {
                                tags.add(AprilTagDetected.parseTag(tag));
                                System.out.println("tag id: " + AprilTagDetected.parseTag(tag).getTagId());
                            } catch (Exception e) {
                                // Handle exception if needed
                            }
                        }
                        currentDetectedAprilTags = tags;
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

    public ArrayList<AprilTagDetected> getCurrentDetectedAprilTags() {
        synchronized (this) {
            // Return a copy to prevent concurrent modification
            return new ArrayList<>(currentDetectedAprilTags);
        }
    }
}