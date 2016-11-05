/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Cake Tank", group = "TeleOp")
//@Disabled
public class CakePushbotTeleopTank_Iterative extends OpMode {

    /* Declare OpMode members. */
    CakeHardwarePushbot robot = new CakeHardwarePushbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double clawOffset = 0.0;                  // Servo mid position
    final double CLAW_SPEED = 0.02;// sets rate to move servo
    static final int COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark Motor Encoder
    private ElapsedTime runtime = new ElapsedTime();
    private boolean forkExtended = false;
    private double runTime;
    int forkMoveDistance = COUNTS_PER_MOTOR_REV * 1;
    static final double FORWARD_SPEED = 0.6;
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position
    double position = (MAX_POS - MIN_POS) / 2;
    double leftDriveMotorPower = 0; //defines variable
    double rightDriveMotorPower = 0;

    // Valid states for the forklift.
    private  enum ForkState {
        extended,
        retracted,
        moving
    }

    // Assume we start retracted.
    ForkState forkState = ForkState.retracted;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap); //Maps hardware
        robot.pushLeft.setPosition(1);
        robot.pushRight.setPosition(-1);
        robot.forkRaise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override

    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        leftDriveMotorPower = -gamepad1.left_stick_y;
        rightDriveMotorPower = -gamepad1.right_stick_y; //Have to offset the negatives
        robot.leftMotor.setPower(leftDriveMotorPower); //Makes the robot run
        robot.rightMotor.setPower(rightDriveMotorPower);
        // Left bumper
        //robot.forkRaise.setPower(FORWARD_SPEED);
        //while (gamepad1.left_bumper && (runtime.seconds() < 3.0))
        // {
/*        telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();*/
        //}
        if (gamepad1.a) {
            robot.pushRight.setPosition(1);
        }
        if (gamepad1.b) {
            robot.pushLeft.setPosition(-1);
        }
        // Extend forklift
        if (gamepad1.left_bumper && !forkExtended) {
            telemetry.addData("Extending forklift!", "");
            robot.forkRight.setPower(.5);//Forklift out
            robot.forkLeft.setPower(.5);//Forklift out
            runtime.reset();
            runTime = 4;
            telemetry.addData("%2.5f S Elapsed", runtime.seconds());
        }
        if (!forkExtended && runTime != 0 && runtime.seconds() >= runTime) {
            robot.forkRight.setPower(0);
            robot.forkLeft.setPower(0);//Stop Motor
            forkExtended = true;
            runTime = 0;
        }
        // Retract forklift
        if (gamepad1.right_bumper && forkExtended) {
            telemetry.addData("Retracting forklift!", "");
            robot.forkRight.setPower(-.5);//Forklift in
            robot.forkLeft.setPower(-.5);//Forklift out
            runtime.reset();
            runTime = 4;
        }
        if (forkExtended && runTime != 0 && runtime.seconds() >= runTime) {
            robot.forkRight.setPower(0);
            robot.forkLeft.setPower(0);//Stop Motor
            forkExtended = false;
            runTime = 0;
        }
        //Raises forklift using encoders
        if (gamepad1.dpad_up) {
            robot.forkRaise.setTargetPosition(forkMoveDistance);//Forklift in
            robot.forkRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.forkRaise.setPower(.5);
        }

        if (gamepad1.dpad_down) {
            //robot.forkRaise.setPower(0);
            robot.forkRaise.setTargetPosition(0);
        }

        telemetry.addData("left drive motor power: ", "%.2f", leftDriveMotorPower);
        telemetry.addData("right drive motor power: ", "%.2f", rightDriveMotorPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
