/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="push sample", group="Robot")
@Disabled
public class autotime_highbasket extends LinearOpMode {

    // This opMode is for an Autonomous drive from the right submersible seam (48 inches from right side wall)
    // and is intended for parking (only) in the observation zone for either red or blue alliance.
    // Insure that no other robots will be in your way for this parking opMode to work as intended.
    private ElapsedTime     runtime = new ElapsedTime();
    public static double     FORWARD_SPEED1 = 0.5;

    public static double    STEP1_TIME = 0.5;

    public static double STEP2_TIME = 2;

    public static double STEP3_TIME = 1.4;

    public static double     TURN_SPEED1 = 0.5;

    public static double STEP4_TIME= 0.4;

    public static double FORWARD_SPEED2 = 0.5;

  public static double  FORWARD_SPEED3;

  public static double STEP5_TIME;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        final DcMotor frontRight  = hardwareMap.get(DcMotor.class, "motor1");
        final DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "motor2");
        final DcMotor backRight = hardwareMap.get(DcMotor.class, "motor3");
        final DcMotor backLeft = hardwareMap.get(DcMotor.class, "motor4");
        final DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();


        // Step 1:  Drive forward for a half second to get off the starting side wall (prevent interference).
        frontLeft.setPower(FORWARD_SPEED1);
        backLeft.setPower(FORWARD_SPEED1);
        frontRight.setPower(FORWARD_SPEED1);
        backRight.setPower(FORWARD_SPEED1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < STEP1_TIME)) {
            telemetry.addData("Path", "step 1: moving forward", runtime.seconds());
            telemetry.update();
        }

        frontLeft.setPower(-TURN_SPEED1);
        backLeft.setPower(TURN_SPEED1);
        frontRight.setPower(-TURN_SPEED1);
        backRight.setPower(-TURN_SPEED1);
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < STEP2_TIME)) {
            telemetry.addData("path", "step 2: turning left", runtime.seconds());
            telemetry.update();

        }


        arm.setPower(0.5);
        sleep(250);
        arm.setPower(0.5);
        sleep(250);
        arm.setPower(0.5);
        sleep(250);
        arm.setPower(0);
        sleep(250);
        arm.setPower(0.5);
        sleep(250);
        arm.setPower(0);
        sleep(250);
        arm.setPower(0.5);
        sleep(250);
        arm.setPower(0);
        arm.setPower(0.5);
        sleep(250);
        arm.setPower(0.5);
        sleep(250);
        arm.setPower(0.5);
        sleep(250);
        arm.setPower(0);
        sleep(250);
        arm.setPower(0.5);
        sleep(250);
        arm.setPower(0);
        sleep(250);
        arm.setPower(0.5);
        sleep(250);
        arm.setPower(0);

        while (opModeIsActive() && (runtime.seconds() < STEP3_TIME)) {
            telemetry.addData("Path", "step 3: extending arm", runtime.seconds());
            telemetry.update();
        }

        frontLeft.setPower(FORWARD_SPEED2);
        backLeft.setPower(FORWARD_SPEED2);
        frontRight.setPower(FORWARD_SPEED2);
        backRight.setPower(FORWARD_SPEED2);
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < STEP4_TIME)) {
            telemetry.addData("Path", "step 4: moving forward", runtime.seconds());
            telemetry.update();
        }

        frontLeft.setPower(TURN_SPEED1);
        backLeft.setPower(TURN_SPEED1);
        frontRight.setPower(-TURN_SPEED1);
        backRight.setPower(-TURN_SPEED1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < STEP5_TIME)) {
            telemetry.addData("Path", "step 5: strafing left", runtime.seconds());
            telemetry.update();
        }
//THIS IS THE DIRECTIONS FOR STRSFING RIGHT.

        //frontLeft.setPower(STRAFE_SPEED);
        //backLeft.setPower(STRAFE_SPEED);
        //frontRight.setPower(-STRAFE_SPEED);
        //ackRight.setPower(-STRAFE_SPEED);

        frontLeft.setPower(FORWARD_SPEED3);
        backLeft.setPower(FORWARD_SPEED3);
        frontRight.setPower(FORWARD_SPEED3);
        backRight.setPower(FORWARD_SPEED3);
        runtime.reset();

        // Use the following operations if driving backwards is desired.
        //  frontLeft.setPower(-FORWARD_SPEED);
        //  backLeft.setPower(-FORWARD_SPEED);
        //  frontRight.setPower(-FORWARD_SPEED);
        //  backRight.setPower(-FORWARD_SPEED);
        //  runtime.reset();
        //  while (opModeIsActive() && (runtime.seconds() < 1.0)) {
        //      telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
        //       telemetry.update();
        //  }

        // Step 4:  Stop the robot.
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // Provide telemetry to FTC Dashboard that can be manipulated.
        while (opModeIsActive()) {

            TelemetryPacket values = new TelemetryPacket();
            values.put("First step time", STEP1_TIME);
            values.put("First step speed", FORWARD_SPEED1);
                values.put("Second step time", STEP2_TIME);
                values.put("Turn speed", TURN_SPEED1);
            values.put("Third step time",STEP3_TIME);
            values.put("Third step speed",FORWARD_SPEED2 );
            values.put("fourth step time",STEP4_TIME );
            FtcDashboard.getInstance().sendTelemetryPacket(values);
        }

        sleep(1000);
    }
}

