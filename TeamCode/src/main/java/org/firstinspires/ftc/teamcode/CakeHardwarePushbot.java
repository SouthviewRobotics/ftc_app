package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Cake robot derived from Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */
class CakeHardwarePushbot {
    /* Public OpMode members. */
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor launchMotor = null;
    public Servo pushRight = null;
    public Servo pushLeft = null;
    public TouchSensor forkStop = null;
    public DeviceInterfaceModule deviceInterfaceModule = null;
    public ColorSensor colorForwardLeft = null;
    public ColorSensor colorForwardRight = null;
    public ColorSensor colorDown = null;
    public OpticalDistanceSensor distanceForward = null;

    /* local OpMode members. */
    private HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public CakeHardwarePushbot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define Motors and servos
        leftMotor = hwMap.dcMotor.get("left drive");
        rightMotor = hwMap.dcMotor.get("right drive");
        launchMotor = hwMap.dcMotor.get("launch");
        pushLeft = hwMap.servo.get("beacon left");
        pushRight = hwMap.servo.get("beacon right");

        // Set sensors
        forkStop = hwMap.touchSensor.get("fork stop");
        deviceInterfaceModule = hwMap.deviceInterfaceModule.get("dim");
        colorForwardLeft = hwMap.colorSensor.get("color forward left");
        colorForwardRight = hwMap.colorSensor.get("color forward right");
//        colorDown = hwMap.colorSensor.get("color down");
//         Color sensors need different addresses.
//        colorDown.setI2cAddress(I2cAddr.create7bit(0x26));

        // Set motor direction to reflect mounting positions.
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        launchMotor.setPower(0);

        // Stop and reset encoders in all motors.
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to required encoder mode.
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

