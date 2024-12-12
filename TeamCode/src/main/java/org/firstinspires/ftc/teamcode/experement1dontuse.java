package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp(name="experement1dontuse", group="Linear OpMode")

public class experement1dontuse extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public static int x;
    public static int y;
    public static int z;
    public static int a;
    public static int b;

    public static int CLAWPOSITION;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        final DcMotor frontRight = hardwareMap.get(DcMotor.class, "motor1");
        final DcMotor frontLeft = hardwareMap.get(DcMotor.class, "motor2");
        final DcMotor backRight = hardwareMap.get(DcMotor.class, "motor3");
        final DcMotor backLeft = hardwareMap.get(DcMotor.class, "motor4");
        final DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        Servo claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        while(opModeIsActive()) {
            TelemetryPacket values = new TelemetryPacket();
            values.put("Forward Time", x);
            values.put("Strafe Time", y);
            values.put("Strafe Time", z);
            values.put("claw", claw.getPosition());

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("arm pos", arm.getCurrentPosition());
            packet.put("arm power", arm.getPower());
            packet.put("claw", claw.getPosition());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
