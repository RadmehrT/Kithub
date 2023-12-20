package frc.utils;

public class Conversions {
    public static double encoderTicksToRPM(double tickRate, double gearRatio) {
        double motorRPM = tickRate * (500.0/2048.0);
        double RPM = motorRPM/gearRatio;
        return RPM;
    }

    public static double RPMtoMPS(double RPM, double circumference) {
        return (RPM * circumference) / 60;
    }
}
