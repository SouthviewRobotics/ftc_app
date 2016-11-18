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
    }

    private Telemetry telemetry = null;
    private Gamepad gamepad1 = null;
    // Seconds to delay before starting opmode.
    private int startDelay = 0;

    public int getStartDelay() {
        return startDelay;
    }

    public void setStartDelay(int startDelay) {
        this.startDelay = startDelay;
    }

    public AllianceColor getAlliance() {
        return alliance;
    }

    public void setAlliance(AllianceColor alliance) {
        this.alliance = alliance;
    }

    public enum AllianceColor {
        None,
        Red,
        Blue
    }

    private AllianceColor alliance = AllianceColor.None;

    public void ShowMenu() {
        while (alliance == AllianceColor.None) {
            if (gamepad1.x) {
                alliance = AllianceColor.Blue;
            }

            if (gamepad1.b) {
                alliance = AllianceColor.Red;
            }

            if (gamepad1.dpad_left) {
                startDelay = startDelay > 0 ? startDelay-- : startDelay;
            }

            if (gamepad1.dpad_right) {
                startDelay = startDelay < 20 ? startDelay++ : startDelay;
            }

            telemetry.addData("Menu", "x for Blue, b for Red, dpad left or right for delay");
            telemetry.addData("Finished", "Press gamepad Start");
            telemetry.addData("Selected", "Alliance %s Delay %d", alliance, startDelay);
            telemetry.update();
            if (gamepad1.start) {
                break;
            }
        }
    }
}
