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

//this is the directory:
//step one:
/*
                                                         ---------------------------->
 */
/* the pourpus of this is to back up the robot eith, or withhout a laoded sample on the back into the human player zone,
 geting it points, or a easy specemen.

 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@Autonomous(name="easy park 2", group="Robot")
@Config
public class easy_park_2 extends LinearOpMode {

    // This opMode is for an Autonomous drive from the right submersible seam (48 inches from right side wall)
    // and is intended for parking (only) in the observation zone for either red or blue alliance.
    // Insure that no other robots will be in your way for this parking opMode to work as intended.
    private ElapsedTime     runtime = new ElapsedTime();
    public static double     FORWARD_SPEED = 0.4;
    public static double     STRAFE_SPEED    = 0.0;
    public static double    timeOne = 1;
    public static double     timeTwo = 0.0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        final DcMotor frontRight  = hardwareMap.get(DcMotor.class, "motor1");
        final DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "motor2");
        final DcMotor backRight = hardwareMap.get(DcMotor.class, "motor3");
        final DcMotor backLeft = hardwareMap.get(DcMotor.class, "motor4");
           final DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
          Servo claw = hardwareMap.get(Servo.class, "claw");


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
        frontLeft.setPower(-FORWARD_SPEED);
        backLeft.setPower(-FORWARD_SPEED);
        frontRight.setPower(-FORWARD_SPEED);
        backRight.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeOne)) {
            telemetry.addData("Path", "Step 1", runtime.seconds());
            telemetry.update();
        }

        // Done. Stop the robot.
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        telemetry.addData("Path", "Complete");
        telemetry.update();

        // Provide telemetry to FTC Dashboard that can be manipulated.
        while (opModeIsActive()) {

            TelemetryPacket values = new TelemetryPacket();
            values.put("Forward Time", timeOne);
            values.put("Strafe Time", timeTwo);
            values.put("Strafe speed",FORWARD_SPEED);
            FtcDashboard.getInstance().sendTelemetryPacket(values);
        }

        sleep(1000);
    }
}

