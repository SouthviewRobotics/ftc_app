package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by Ron on 10/31/2016.
 * Collection of functions for use in any opmode.
 */
public class UtilityFunctions {
    public UtilityFunctions() {
    }

    // For convenience when passing beacon colors around.
    public enum BeaconColor {
        Red,
        Blue
    }

    /**
     * Scale a joystick value to smooth it for motor setting.
     * The cube results in finer control at slow speeds.
     */
    public static double ScaleMotorCube(double joyStickPosition) {
        return (double) Math.pow(joyStickPosition, 3.0);
    }

    /**
    * Scale the joystick value to smooth it for motor settings.
     * This algorithm gives a bit more sensitivity than the ScaleMotorCube() method.
    * */
    public static double ScaleMotorTan(double joyStickPosition) {
        return (double) ((joyStickPosition / 1.07) * (.62 * (joyStickPosition * joyStickPosition)) + .45);
    }

    /**
     * Scale the joystick input using a nonlinear algorithm.
     * Tweak the array to get the curve you need.
     */
    public double ScaleMotorLookTable(double joyStickPosition) {
        //
        // Assume no scaling.
        //
        double lScale;

        //
        // Ensure the values are legal.
        //
        double lPower = Range.clip(joyStickPosition, -1, 1);

        double[] lArray =
                {0.00d, 0.05d, 0.09d, 0.10d, 0.12d
                        , 0.15d, 0.18d, 0.24d, 0.30d, 0.36d
                        , 0.43d, 0.50d, 0.60d, 0.72d, 0.85d
                        , 1.00d, 1.00d
                };

        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int lIndex = (int) (lPower * 16.0);
        if (lIndex < 0) {
            lIndex = -lIndex;
        } else if (lIndex > 16) {
            lIndex = 16;
        }

        if (lPower < 0) {
            lScale = -lArray[lIndex];
        } else {
            lScale = lArray[lIndex];
        }

        return lScale;

    } // ScaleMotorLookTable

}
