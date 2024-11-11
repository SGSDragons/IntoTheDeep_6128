package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="Auto Test", group="Robot")
// @Disabled
public class AutoTest extends LinearOpMode{

    // Declare the motors
    DcMotor leftFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightFrontMotor;
    DcMotor rightBackMotor;

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

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Moving Forward");
            telemetry.update();

    // Test the robot's movement by driving forward

            telemetry.addData("Status", "Moving Forward");
            telemetry.update();

    // Set all motors to run forward
            leftFrontMotor.setPower(0.5);
            leftBackMotor.setPower(0.5);
            rightFrontMotor.setPower(0.5);
            rightBackMotor.setPower(0.5);

    // Wait for 2 seconds (adjust based on testing)
    sleep(2000);

    // Stop the robot
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);

    // Pause and then reverse
    sleep(1000);

    // Test reverse direction by driving backward
            telemetry.addData("Status", "Moving Backward");
            telemetry.update();

    // Set all motors to run in reverse
            leftFrontMotor.setPower(-0.5);
            leftBackMotor.setPower(-0.5);
            rightFrontMotor.setPower(-0.5);
            rightBackMotor.setPower(-0.5);

    // Wait for 2 seconds (adjust based on testing)
    sleep(2000);

    // Stop the robot
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);

            telemetry.addData("Status", "Test Complete");
            telemetry.update();
}
    }

}
