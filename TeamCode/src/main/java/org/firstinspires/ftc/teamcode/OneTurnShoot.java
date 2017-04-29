package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name = "reboot shooter", group = "corner")

public class OneTurnShoot extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Servo shooterservoX = hardwareMap.servo.get("shooterservox");

        TouchSensor shootertouch = hardwareMap.touchSensor.get("shootertouch");

        shooterservoX.setPosition(0.15);
        sleep(200);
        while (opModeIsActive() && !shootertouch.isPressed()){
            idle();
        }
        shooterservoX.setPosition(0.5);

        while (opModeIsActive()){
            if (gamepad1.dpad_up){
                shooterservoX.setPosition(0.4);
            } else {
                shooterservoX.setPosition(0.5);
            }
        }




    }

}
