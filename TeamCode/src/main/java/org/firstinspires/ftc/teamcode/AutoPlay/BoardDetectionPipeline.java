package org.firstinspires.ftc.teamcode.AutoPlay;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import java.util.*;

public class BoardDetectionPipeline extends OpenCvPipeline {

    public static boolean active = false;

    public BoardDetectionPipeline(){
        active = false;
    }
    @Override
    public Mat processFrame(Mat input) {
        return null;
    }

    public static void main(String[] args) {
        // Assume the board has a fixed size (rows x cols) or compute it dynamically
        int rows = 7; // Example value
        int cols = 3; // Example value
        double[][][] boardColors = new double[rows][cols][3]; // Store BGR colors

        Mat image = Imgcodecs.imread("path_to_your_image.jpg");
        Mat grayImage = new Mat();
        Mat thresholdedImage = new Mat();

        // Convert to grayscale and threshold the image
        Imgproc.cvtColor(image, grayImage, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(grayImage, thresholdedImage, 50, 255, Imgproc.THRESH_BINARY);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresholdedImage, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Process each contour
        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            // Make sure the contour is of adequate size to be a hexagon
            if (boundingRect.width > 10 && boundingRect.height > 10) { // Example condition
                Point samplePoint = getSamplePoint(boundingRect);
                int row = calculateRow(boundingRect, rows, image.height());
                int col = calculateColumn(boundingRect, cols, image.width());
                if (row >= 0 && row < rows && col >= 0 && col < cols) {
                    double[] color = image.get((int) samplePoint.y, (int) samplePoint.x);
                    boardColors[row][col] = color;
                }
            }
        }

        // The boardColors array now contains the color of each hexagon
    }

    private static Point getSamplePoint(Rect boundingRect) {
        // Calculate a sample point that's inside the edge of the hexagon, not at the center
        double offsetX = boundingRect.width * 0.1;
        double offsetY = boundingRect.height * 0.1;
        return new Point(boundingRect.x + offsetX, boundingRect.y + offsetY);
    }

    private static int calculateRow(Rect boundingRect, int totalRows, int imageHeight) {
        // Implement logic to calculate row index based on the position of the hexagon
        // This is a placeholder function and would need actual logic to calculate the row
        return boundingRect.y * totalRows / imageHeight; // Simplified example
    }

    private static int calculateColumn(Rect boundingRect, int totalCols, int imageWidth) {
        // Implement logic to calculate column index based on the position of the hexagon
        // This is a placeholder function and would need actual logic to calculate the column
        return boundingRect.x * totalCols / imageWidth; // Simplified example
    }
}

