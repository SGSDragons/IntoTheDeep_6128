package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Util;

@TeleOp(name="Teleop_6128_v2", group="Linear OpMode")
//@Disabled
@Config
public class Teleop_6128_v2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public static double ARM_POWER = 1050;
    public static double VERTICAL_ARM_OFFSET = 650.0;
    public static double FEED_FORWARD_GAIN = 250;

    public static double ARM_P = 15.0;
    public static double ARM_I = 1;
    public static double ARM_D = 0;
    public static double ARM_F = 0;

    class ArmDriver {

        // The angle of the arm when resting on the ground
        public static final double GROUND_ANGLE = -30.0;

        // How many ticks to go from resting on the ground to being vertical

        private int groundPos = 0;
        public void resetGround(int pos) {
            groundPos = pos;
        }

        public double getArmAngle(int armPosition) {
            return ((armPosition-groundPos) / VERTICAL_ARM_OFFSET) * (90.0 - GROUND_ANGLE) + GROUND_ANGLE;
        }

    }

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
        final DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        int mode;

        PIDFCoefficients coefficients = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        coefficients.p = ARM_P;
        coefficients.i = ARM_I;
        coefficients.d = ARM_D;
        coefficients.f = ARM_F;
        arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmDriver armDriver = new ArmDriver();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double max;

             double speed = 1.0;

             if(gamepad1.left_trigger > 0){
                 speed = 0.5;
             }

            if(gamepad1.right_trigger > 0){
                speed = 1.5;
            }

            double drive = -gamepad1.left_stick_y * speed;//drive
            double lateral = gamepad1.left_stick_x * speed;//turn
            double turn  =  gamepad1.right_stick_x;//strafe

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

            armPower = Math.pow(gamepad2.left_stick_y, 3.0)*ARM_POWER;

            if (gamepad2.left_bumper) {
                armPower = armPower * 2;
            }

            if (gamepad2.x) {
                armDriver.resetGround(arm.getCurrentPosition());
            }

            TelemetryPacket packet = new TelemetryPacket();

            if (gamepad2.y) {
                coefficients.p = ARM_P;
                coefficients.i = ARM_I;
                coefficients.d = ARM_D;
                coefficients.f = ARM_F;
                arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            packet.put("arm pos", arm.getCurrentPosition());
            packet.put("arm angle", armDriver.getArmAngle(arm.getCurrentPosition()));
            packet.put("arm power", arm.getPower());
            packet.put("arm mode", arm.getMode().toString());
            packet.put("claw", claw.getPosition());

            double armAngle = armDriver.getArmAngle(arm.getCurrentPosition());
            double gravityTorque = FEED_FORWARD_GAIN * Math.cos(armAngle / 360.0 * (2*Math.PI));
            packet.put("gravity", gravityTorque);
            packet.put("velocity", arm.getVelocity());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            if (Math.abs(armPower) > 0.02) {
                armPower += gravityTorque;
                arm.setVelocity(armPower);
            } else {
                arm.setVelocity(gravityTorque/2.0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}
