package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

public class AprilTagDetected {
    private int tagId;
    private double[] centerCoordinates;
    private double[][] corners;

    public AprilTagDetected(int tagId, double[] centerCoordinates, double[][] corners) {
        this.tagId = tagId;
        this.centerCoordinates = centerCoordinates;
        this.corners = corners;
    }

    public int getTagId() {
        return tagId;
    }

    public double[] getCenterCoordinates() {
        return centerCoordinates;
    }

    public double[][] getCorners() {
        return corners;
    }

    @Override
    public String toString() {
        return "AprilTag{" +
                "tagId=" + tagId +
                ", centerCoordinates=" + Arrays.toString(centerCoordinates) +
                ", corners=" + Arrays.deepToString(corners) +
                '}';
    }

    public static AprilTagDetected parseTag(String input) {
        // Extract tag ID
        int tagId = Integer.parseInt(input.split("Detected tag ID: ")[1].split(",")[0].trim());

        // Extract center coordinates
        String centerPart = input.split("Center coordinates: ")[1].split("\\)")[0].trim();
        centerPart = centerPart.replace("(", "").replace(")", "");
        String[] centerValues = centerPart.split(", ");
        double[] centerCoordinates = new double[2];
        centerCoordinates[0] = Double.parseDouble(centerValues[0]);
        centerCoordinates[1] = Double.parseDouble(centerValues[1]);

        // Extract corners
        String cornersPart = input.split("Corners: ")[1].trim();
        String[] cornerPairs = cornersPart.split("\\]\\s*\\[");
        double[][] corners = new double[4][2];
        for (int i = 0; i < cornerPairs.length; i++) {
            cornerPairs[i] = cornerPairs[i].replace("[", "").replace("]", "").trim();
            String[] cornerValues = cornerPairs[i].split("\\s+");
            corners[i][0] = Double.parseDouble(cornerValues[0]);
            corners[i][1] = Double.parseDouble(cornerValues[1]);
        }

        return new AprilTagDetected(tagId, centerCoordinates, corners);
    }

    // public static ArrayList<AprilTagDetected> parseMultipleTags(String input) {
    //     // Split the input string into individual tag strings using the delimiter
    //     String[] tagStrings = input.split("-next-detection-");
    //     ArrayList<String> individualTags = new ArrayList<>();
    //     for (String tag : tagStrings) {
    //         if (!tag.trim().isEmpty()) {
    //             individualTags.add(tag.trim());
    //         }
    //     }

    //     // Parse each tag sequentially
    //     ArrayList<AprilTagDetected> tags = new ArrayList<>();
    //     for (String tag : individualTags) {
    //         tags.add(parseTag(tag));
    //     }

    //     return tags;
    // }


}