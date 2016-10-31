package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by Ron on 10/31/2016.
 * Collection of functions for use in any opmode.
 */
public class UtilityFunctions {
    public UtilityFunctions() {
    }

    /**
     * Scale a joystick value to smooth it for motor setting.
     * The cube results in finer control at slow speeds.
     */
    public float ScaleMotorCube(float joyStickPosition) {
        return (float) Math.pow(joyStickPosition, 3.0);
    }

    /**
    * Scale the joystick value to smooth it for motor settings.
     * This algorithm gives a bit more sensitivity than the ScaleMotorCube() method.
    * */
    public float ScaleMotorTan(float joyStickPosition) {
        return (float) ((joyStickPosition / 1.07) * (.62 * (joyStickPosition * joyStickPosition)) + .45);
    }

    /**
     * Scale the joystick input using a nonlinear algorithm.
     * Tweak the array to get the curve you need.
     */
    public float ScaleMotorLookTable(float joyStickPosition) {
        //
        // Assume no scaling.
        //
        float lScale;

        //
        // Ensure the values are legal.
        //
        float lPower = Range.clip(joyStickPosition, -1, 1);

        float[] lArray =
                {0.00f, 0.05f, 0.09f, 0.10f, 0.12f
                        , 0.15f, 0.18f, 0.24f, 0.30f, 0.36f
                        , 0.43f, 0.50f, 0.60f, 0.72f, 0.85f
                        , 1.00f, 1.00f
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
