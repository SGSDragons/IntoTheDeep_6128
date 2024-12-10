package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp (name="ServoTester", group="Tests")
@Config
@Disabled
public class ServoTester extends LinearOpMode {

    public static double OPEN = 0.0;
    public static double CLOSE = 1.0;
   @Override
   public void runOpMode() {

       Servo claw = hardwareMap.get(Servo.class, "claw");
       claw.setPosition(0);

       waitForStart();
       claw.getDirection();

       while (opModeIsActive()) {

           if (gamepad1.x) {
               claw.setPosition(OPEN);
           }
           if (gamepad1.y) {
               claw.setPosition(CLOSE);
           }

           TelemetryPacket packet = new TelemetryPacket();
           packet.put("claw", claw.getPosition());
           FtcDashboard.getInstance().sendTelemetryPacket(packet);

       }
   }
}
