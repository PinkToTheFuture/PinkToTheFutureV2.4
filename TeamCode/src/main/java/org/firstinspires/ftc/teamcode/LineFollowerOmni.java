package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="LineFollower", group="PinktotheFuture")
@Disabled
public class LineFollowerOmni extends LinearOpMode {

    public void FollowLine(double pwr, double threshold) throws InterruptedException {

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);


        LightSensor Rlight = hardwareMap.lightSensor.get("Rlight");
        LightSensor Llight = hardwareMap.lightSensor.get("Llight");

        Llight.enableLed(true);
        Rlight.enableLed(true);


        while (opModeIsActive()) {
            waitOneFullHardwareCycle();

            telemetry.addData("Rlight", Rlight.getRawLightDetected());
            telemetry.update();



            LFdrive.setPower(pwr);
            RBdrive.setPower(pwr);
            LBdrive.setPower(-pwr);
            RFdrive.setPower(-pwr);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        FollowLine(0.3, 2.05);

    }
}