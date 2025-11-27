package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorIndexer {

    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public Telemetry telemetry;
    private DetectedColor lastColor = DetectedColor.UNKNOWN;
    private int stableCount = 0;
    private static final int REQUIRED_STABLE_READS = 3;
    public void initcolorsensor(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "SenzorIntakeCH");
        colorSensor.setGain(100);
    }
    public DetectedColor getDetectedColor() {
        DetectedColor current = detectRawColor();

        if (current == lastColor && current != DetectedColor.UNKNOWN) {
            stableCount++;
            if (stableCount >= REQUIRED_STABLE_READS) {
                return current; // confirmat
            }
        } else {
            stableCount = 0;
        }

        lastColor = current;
        return DetectedColor.UNKNOWN;
    }
    private DetectedColor detectRawColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        double a = colors.alpha;

        if (a > 0.8) {
            return DetectedColor.UNKNOWN;
        }

        double r = colors.red / a;
        double g = colors.green / a;
        double b = colors.blue / a;

        double[] PURPLE = {1.13, 1.589, 1.743};
        double[] GREEN  = {0.9, 2.1, 1.6};

        double distPurple = dist(r, g, b, PURPLE);
        double distGreen  = dist(r, g, b, GREEN);

        double threshold = 0.45;

        if (distPurple < distGreen && distPurple < threshold)
            return DetectedColor.PURPLE;

        if (distGreen < distPurple && distGreen < threshold)
            return DetectedColor.GREEN;

        return DetectedColor.UNKNOWN;
    }
    private double dist(double r, double g, double b, double[] ref) {
        return Math.sqrt(
                Math.pow(r - ref[0], 2) +
                        Math.pow(g - ref[1], 2) +
                        Math.pow(b - ref[2], 2)
        );
    }

//    public DetectedColor getDetectedColor() {
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//
//        float normred, normgreen, normblue;
//        normred = colors.red / colors.alpha;
//        normgreen = colors.green / colors.alpha;
//        normblue = colors.blue / colors.alpha;
//
////        if( normred > 0.8 && normgreen > 1.0739 && normblue > 1.3866 && normred < 1.46 && normgreen < 2.104 && normblue < 2.1 && normblue > normgreen){
////            return DetectedColor.PURPLE;
////
////        } else if (normred > 0.5 && normgreen > 1.4 && normblue > 1.1 && normred < 1.3 && normgreen < 2.8 && normblue < 2.1 && normgreen > normblue){
////            return DetectedColor.GREEN;
////        }else {
////            return DetectedColor.UNKNOWN;
////        }
//}

}