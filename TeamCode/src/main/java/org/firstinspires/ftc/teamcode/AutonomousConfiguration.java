package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Ron on 11/16/2016.
 * This class provides a way to configure you autonomous opmode.
 * Current version allows selecting:
 * - Alliance
 * - Start Delay
 */

public class AutonomousConfiguration {
    public AutonomousConfiguration(Gamepad gamepad, Telemetry telemetry1) {
        this.gamepad1 = gamepad;
        this.telemetry = telemetry1;
        alliance = AllianceColor.None;
        // Default selections if driver does not select any.
        startPosition = StartPosition.Center;
        parkLocation = ParkLocation.Ramp;
        pressBeacon = PressBeacon.No;
    }

    private Telemetry telemetry = null;
    private Gamepad gamepad1 = null;
    // Seconds to delay before starting opmode.
    private int startDelay = 0;

    public int getStartDelay() {
        return startDelay;
    }

    public AllianceColor getAlliance() {
        return alliance;
    }

    public StartPosition getStartPosition() {
        return startPosition;
    }

    public ParkLocation getParkLocation() {
        return parkLocation;
    }

    public PressBeacon getPressBeacon() {
        return pressBeacon;
    }

    // Where do we place the robot
    public enum StartPosition {
        Center,
        Left;

        public StartPosition getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    // Where do we park
    public enum ParkLocation {
        Center,
        Ramp
    }

    // Press a beacon button
    public enum PressBeacon {
        No,
        Yes
    }

    public enum AllianceColor {
        None,
        Red,
        Blue
    }

    private AllianceColor alliance;
    private StartPosition startPosition;
    private ParkLocation parkLocation;
    private PressBeacon pressBeacon;

    public void ShowMenu() {
        //
        do {
            if (gamepad1.x) {
                alliance = AllianceColor.Blue;
            }

            if (gamepad1.b) {
                alliance = AllianceColor.Red;
            }

            if (gamepad1.dpad_left && startDelay > 0) {
                startDelay--;
            }

            if (gamepad1.dpad_right && startDelay < 20) {
                startDelay++;
            }

            if (gamepad1.y) {
                startPosition = startPosition.getNext();
            }

            if (gamepad1.start && alliance != AllianceColor.None) {
                break;
            }

            telemetry.addData("Menu", "x for Blue, b for Red, dpad left or right for delay");
            telemetry.addData("", "y to cycle start position");
            telemetry.addData("Finished", "Press gamepad Start");
            telemetry.addData("Selected", "Alliance %s Delay %d", alliance, startDelay);
            telemetry.addData("", "Start position %s", startPosition);
            telemetry.update();
            // Slow down so the delay incrementing works better
            sleep(500);
        } while (true);
    }

    private void sleep(long milliseconds)
    {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
