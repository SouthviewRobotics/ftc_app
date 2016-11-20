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
 * This OpMode uses the CakeHardwarePushbot class to define the devices on the robot.
 * All device access is managed through the CakeHardwarePushbot class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the Winch using the Dpad up and Dpad down respectively.
 * It also extends and retracts the forklift extenders using the left and right Bumper buttons.
 * <p>
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Cake Tank", group = "TeleOp")
//@Disabled
public class CakePushbotTeleopTank_Iterative extends OpMode {

    /* Declare OpMode members. */
    private CakeHardwarePushbot robot = new CakeHardwarePushbot(); // use the class created to define a Pushbot's hardware
    private static final int COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark Motor Encoder
    //  private static int liftAngle = 180;
    private ElapsedTime runtime = new ElapsedTime();
    private int forkRaiseDistance = 640;
    private boolean forkRaised = false;

    // Valid states for the forklift.
    private enum ForkState {
        extended,
        retracted,
        moving
    }

    // Assume we start retracted.
    private ForkState forkState = ForkState.retracted;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap); //Maps hardware
        robot.pushLeft.setPosition(-1);
        robot.pushRight.setPosition(1);
        robot.forkRaise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Make sure the forklift is retracted.
        RetractForkLift();
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
        double leftDriveMotorPower = -gamepad1.left_stick_y;
        double rightDriveMotorPower = -gamepad1.right_stick_y;
        robot.leftMotor.setPower(leftDriveMotorPower); //Makes the robot run
        robot.rightMotor.setPower(rightDriveMotorPower);
        /*
           The following code could be used to test the servos, if needed,
           otherwise keep it as a comment.
        */

//        if (gamepad1.a) {
//            robot.pushRight.setPosition(1);
//        }
//        if (gamepad1.b) {
//            robot.pushLeft.setPosition(-1);
//        }

        /*
        Forklift operation rules:
            Initially positioned to retracted by init()
            Can only be extended if already retracted
            Can only be retracted if already extended
            Can only be raised or lowered if extended
            Cannot be retracted if extended
         */

        // Extend forklift (no encoders on these motors, user time)
        if (gamepad2.left_bumper && ForkState.retracted == forkState) {
            robot.forkRight.setPower(.5);   //Forklift out
            robot.forkLeft.setPower(.5);    //Forklift out
            runtime.reset();
            forkState = ForkState.moving;
        }

        // Retract forklift (no encoders on these motors, user time)
        if (gamepad2.right_bumper && ForkState.extended == forkState && !forkRaised) {
            robot.forkRight.setPower(-.5);  //Forklift in
            robot.forkLeft.setPower(-.5);   //Forklift in
            runtime.reset();
            forkState = ForkState.moving;
        }

        // Stop forklifts when time has been reached
        double SECONDS_TO_EXTEND = 4;
        if (ForkState.moving == forkState && runtime.seconds() >= SECONDS_TO_EXTEND) {
            if (robot.forkLeft.getPower() >= 0) {
                forkState = ForkState.extended;
            } else {
                forkState = ForkState.retracted;
            }
            //Stop motors
            robot.forkRight.setPower(0);
            robot.forkLeft.setPower(0);
        }

        if (forkState == ForkState.extended) {
            //Raises forklift using encoders
            if (gamepad2.dpad_up) {
                robot.forkRaise.setTargetPosition(forkRaiseDistance);
                robot.forkRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.forkRaise.setPower(.5);
                forkRaised = true;
            }

            if (gamepad2.dpad_down) {
                robot.forkRaise.setTargetPosition(0);
                robot.forkRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.forkRaise.setPower(.5);
                forkRaised = false;
            }
        }

        telemetry.addData("ForkState", "%s", forkState);
        telemetry.addData("Raise (target, current)", "%d %d", robot.forkRaise.getTargetPosition(), robot.forkRaise.getCurrentPosition());
        telemetry.addData("left drive motor power: ", "%.2f", leftDriveMotorPower);
        telemetry.addData("right drive motor power: ", "%.2f", rightDriveMotorPower);
    }

    // Use this at init to make sure the forklift is retracted. Uses the touch sensor.
    public void RetractForkLift() {
        telemetry.addData("Touch", "%s pressed = %s", robot.forkStop.getDeviceName(), robot.forkStop.isPressed());
        runtime.reset();
        while (!robot.forkStop.isPressed() && runtime.seconds() <= 5) {
            robot.forkLeft.setPower(-.1);
            robot.forkRight.setPower(-.1);
        }

        robot.forkLeft.setPower(0);
        robot.forkRight.setPower(0);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
