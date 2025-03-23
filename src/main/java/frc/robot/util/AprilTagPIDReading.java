package frc.robot.util;

import java.net.DatagramPacket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;


public class AprilTagPIDReading {
    private double pidX;
    private double pidZ;
    private double pidYaw;
    private double metersX;
    private double metersY;
    private double tagRotation;

    public AprilTagPIDReading(double pidX, double pidZ, double pidYaw, double metersX, double metersY, double tagRotation) {
        this.pidX = pidX;
        this.pidZ = pidZ;
        this.pidYaw = pidYaw;
        this.metersX = metersX;
        this.metersY = metersY;
        this.tagRotation = tagRotation;

    }

    public static AprilTagPIDReading decompress(DatagramPacket packet) {
        double pidX = 0.0, pidZ = 0.0, pidYaw = 0.0, metersX = 0.0, metersY = 0.0, tagRotation = 0.0;



        // String[] parts = compressedReading.split(",");
        // for (String part : parts) {
        //     part = part.trim(); // remove any leading/trailing whitespaces
        //     if (part.startsWith("PID X:")) {
        //         pidX = Double.parseDouble(part.substring(6).trim());
        //     } else if (part.startsWith("PID Z:")) {
        //         pidZ = Double.parseDouble(part.substring(6).trim());
        //     } else if (part.startsWith("PID Yaw:")) {
        //         pidYaw = Double.parseDouble(part.substring(8).trim());
        //     }
        // }
        ByteBuffer buffer = ByteBuffer.wrap(packet.getData());

        buffer.order(ByteOrder.LITTLE_ENDIAN);

        pidX = buffer.getFloat();
        pidZ = buffer.getFloat();
        pidYaw = buffer.getFloat();
        metersX = buffer.getFloat();
        metersY = buffer.getFloat();
        tagRotation = buffer.getFloat();



        return new AprilTagPIDReading(pidX, pidZ, pidYaw, metersX, metersY, tagRotation);
    }

    public double getPidX() {
        return pidX;
    }

    public double getPidZ() {
        return pidZ;
    }

    public double getPidYaw() {
        return pidYaw;
    }

    public double getMetersX() {
        return metersX;
    }

    public double getMetersY() {
        return metersY;
    }

    public double getTagRotation() {
        return tagRotation;
    }

    @Override
    public String toString() {
        return "AprilTagPIDReading [pidX=" + pidX + ", pidZ=" + pidZ + 
               ", pidYaw=" + pidYaw + ", metersX=" + metersX + 
               ", metersY=" + metersY + ", tagRotation=" + tagRotation + "]";
    }
}
