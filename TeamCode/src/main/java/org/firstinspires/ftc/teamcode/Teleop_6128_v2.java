package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleop_6128_v2", group="Linear OpMode")
//@Disabled
public class Teleop_6128_v2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public static int ARM_LOW = 35;
    public static int ARM_MEDIUM = 200;
    public static int ARM_HIGH = 350;
    public static double ARM_POWER = 0.5;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        final DcMotor frontRight  = hardwareMap.get(DcMotor.class, "motor1");
        final DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "motor2");
        final DcMotor backRight = hardwareMap.get(DcMotor.class, "motor3");
        final DcMotor backLeft = hardwareMap.get(DcMotor.class, "motor4");
        final DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double max;

            double drive = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;

            double leftFrontPower = drive + lateral + turn;
            double leftBackPower = drive - lateral + turn;
            double rightFrontPower = drive - lateral - turn;
            double rightBackPower = drive + lateral - turn;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            frontLeft.setPower(leftFrontPower/1.5);
            backLeft.setPower(leftBackPower/1.5);
            frontRight.setPower(rightFrontPower/1.5);
            backRight.setPower(rightBackPower/1.5);

           //The following code controls the extendable lever arm for the claw.

            double armPower = 0.0;

            claw.setPosition(gamepad2.right_trigger);

            if (gamepad2.y) {
                arm.setTargetPosition(ARM_HIGH);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armPower = ARM_POWER;
            }
            else if (gamepad2.b) {
                arm.setTargetPosition(ARM_MEDIUM);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                armPower = ARM_POWER;
            }
            else if (gamepad2.a) {
                arm.setTargetPosition(ARM_LOW);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                armPower = ARM_POWER;
            }
            else {
                armPower = gamepad2.left_stick_y*ARM_POWER;
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (gamepad2.left_bumper) {
                    armPower = armPower * 2;
                }
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("arm pos", arm.getCurrentPosition());
            packet.put("arm power", arm.getPower());
            packet.put("arm mode", arm.getMode().toString());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            if (Math.abs(armPower) > 0.1) {
                armPower += 0.5*Math.cos(arm.getCurrentPosition()/300.0 * Math.PI/2);
                arm.setPower(armPower);
            } else {
                arm.setPower(0.0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}
