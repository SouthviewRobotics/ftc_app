package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *

 */
class CakeHardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor forkRight = null;
    public DcMotor forkLeft = null;
    public Servo    pushRight    = null;
    public Servo    pushLeft   = null;
    public DcMotor  forkRaise = null;
    public TouchSensor forkStop = null;

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public CakeHardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left drive");
        rightMotor  = hwMap.dcMotor.get("right drive");
        pushLeft = hwMap.servo.get("beacon left");
        pushRight = hwMap.servo.get("beacon right");
        forkRight = hwMap.dcMotor.get("forklift right");
        forkLeft = hwMap.dcMotor.get("forklift left");
        forkRaise = hwMap.dcMotor.get("fork raise");
        forkStop = hwMap.touchSensor.get("fork stop");

        //armMotor    = hwMap.dcMotor.get("left_arm");
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        forkLeft.setDirection(DcMotor.Direction.FORWARD);
        forkRight.setDirection(DcMotor.Direction.REVERSE);
        forkRaise.setDirection(DcMotor.Direction.REVERSE);
        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        forkLeft.setPower(0);
        forkRight.setPower(0);
        forkRaise.setPower(0);
        //armMotor.setPower(0);
        forkRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forkLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forkRaise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set senors
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        forkLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        forkRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        forkRaise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //TODO Move the servo initialize from the TeleOp to here.
        // Define and initialize ALL installed servos.
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

