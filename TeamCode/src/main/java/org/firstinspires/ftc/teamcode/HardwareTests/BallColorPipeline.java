package org.firstinspires.ftc.teamcode.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
public class BallColorPipeline extends OpenCvPipeline {

    private String colorSequence = "";

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Interval MOV (HSV)
        Scalar lowerPurple = new Scalar(120, 50, 50);
        Scalar upperPurple = new Scalar(160, 255, 255);

        // Interval VERDE (HSV)
        Scalar lowerGreen = new Scalar(35, 60, 60);
        Scalar upperGreen = new Scalar(85, 255, 255);

        Mat maskPurple = new Mat();
        Mat maskGreen = new Mat();

        Core.inRange(hsv, lowerPurple, upperPurple, maskPurple);
        Core.inRange(hsv, lowerGreen, upperGreen, maskGreen);

        // Combina ambele pentru afisare
        Mat combinedMask = new Mat();
        Core.bitwise_or(maskPurple, maskGreen, combinedMask);

        // Detecteaza contururi mov
        List<MatOfPoint> contoursPurple = new ArrayList<>();
        Imgproc.findContours(maskPurple, contoursPurple, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Detecteaza contururi verzi
        List<MatOfPoint> contoursGreen = new ArrayList<>();
        Imgproc.findContours(maskGreen, contoursGreen, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Lista finală pentru bile (pozitie + culoare)
        List<Ball> balls = new ArrayList<>();

        for (MatOfPoint contour : contoursPurple) {
            double area = Imgproc.contourArea(contour);
            if (area > 500) {
                Rect rect = Imgproc.boundingRect(contour);
                balls.add(new Ball(rect.x + rect.width / 2.0, 'P'));
            }
        }

        for (MatOfPoint contour : contoursGreen) {
            double area = Imgproc.contourArea(contour);
            if (area > 500) {
                Rect rect = Imgproc.boundingRect(contour);
                balls.add(new Ball(rect.x + rect.width / 2.0, 'G'));
            }
        }

        // Sortează bilele de la stânga la dreapta
        Collections.sort(balls, Comparator.comparingDouble(b -> b.x));

        // Construieste string-ul final
        StringBuilder sequenceBuilder = new StringBuilder();
        for (Ball b : balls) {
            sequenceBuilder.append(b.color);
        }

        colorSequence = sequenceBuilder.toString();

        // Desenează cercuri pentru debugging
        for (Ball b : balls) {
            Scalar color = (b.color == 'P') ? new Scalar(255, 0, 255) : new Scalar(0, 255, 0);
            Imgproc.circle(input, new Point(b.x, input.rows() / 2.0), 20, color, 3);
        }

        return input;
    }

    public String getColorSequence() {
        return colorSequence;
    }

    private static class Ball {
        double x;
        char color;
        Ball(double x, char color) {
            this.x = x;
            this.color = color;
        }
    }
}

