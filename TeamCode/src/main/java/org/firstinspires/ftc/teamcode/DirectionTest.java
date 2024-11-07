package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DirectionTest extends LinearOpMode {

    // Declare the motors
    DcMotor leftFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightFrontMotor;
    DcMotor rightBackMotor;
    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        leftFrontMotor = hardwareMap.get(DcMotor.class, "motor2");
        leftBackMotor = hardwareMap.get(DcMotor.class, "motor4");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "motor1");
        rightBackMotor = hardwareMap.get(DcMotor.class, "motor3");

        // Set the direction of the motors (if necessary)
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);  // Right side motors are reversed
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);   // Right side motors are reversed

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Test each wheel with a button individually to see if you really know which
            // wheel is actually left Front, etc.

            if (gamepad1.a) {
                leftFrontMotor.setPower(0.5);
            } else {
                leftFrontMotor.setPower(0);
            }
            if (gamepad1.b) {
                leftBackMotor.setPower(0.5);
            } else {
                leftBackMotor.setPower(0);
            }
            if (gamepad1.x) {
                rightFrontMotor.setPower(0.5);
            } else {
                rightFrontMotor.setPower(0);
            }
            if (gamepad1.y) {
                rightBackMotor.setPower(0.5);
            } else {
                rightBackMotor.setPower(0);
            }
        }
    }
}