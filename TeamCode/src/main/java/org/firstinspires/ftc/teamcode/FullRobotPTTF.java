package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Full Robot Omni Drive", group="PinktotheFuture")


public class FullRobotPTTF extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double LFpower = 0;
        double LBpower = 0;
        double RFpower = 0;
        double RBpower = 0;
        double fastency = 1;
        double geleiderPower = 0;
        double sweeperPower = 0;



        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        DcMotor sweeper = hardwareMap.dcMotor.get("sweeper");
        DcMotor geleider = hardwareMap.dcMotor.get("geleider");




        //RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.dpad_up)     fastency = 1;
            if (gamepad1.dpad_down)   fastency = 0.3;
            RFpower = 0;
            RBpower = 0;
            LFpower = 0;
            LBpower = 0;
            geleiderPower = 0;
            sweeperPower = 0;
            if (gamepad1.left_stick_x >= 0 && gamepad1.left_stick_y < 0){
                LFpower = 1;
                LBpower = Math.abs(gamepad1.left_stick_y) - gamepad1.left_stick_x;
                RBpower = 1;
                RFpower = Math.abs(gamepad1.left_stick_y) - gamepad1.left_stick_x;
            }
            if (gamepad1.left_stick_x > 0 && gamepad1.left_stick_y >= 0){
                LFpower = gamepad1.left_stick_x - gamepad1.left_stick_y;
                LBpower = -1;
                RBpower = gamepad1.left_stick_x - gamepad1.left_stick_y;
                RFpower = -1;
            }
            if (gamepad1.left_stick_x <= 0 && gamepad1.left_stick_y > 0){
                LFpower = -1;
                LBpower = -gamepad1.left_stick_y + Math.abs(gamepad1.left_stick_x);
                RBpower = -1;
                RFpower = -gamepad1.left_stick_y + Math.abs(gamepad1.left_stick_x);
            }
            if (gamepad1.left_stick_x < 0 && gamepad1.left_stick_y <= 0){
                LFpower = gamepad1.left_stick_x + Math.abs(gamepad1.left_stick_y);
                LBpower = 1;
                RBpower = gamepad1.left_stick_x + Math.abs(gamepad1.left_stick_y);
                RFpower = 1;
            }

            //RIGHT STICK
            RFpower = RFpower - (gamepad1.right_stick_x);
            RBpower = RBpower - (gamepad1.right_stick_x);
            LFpower = LFpower + (gamepad1.right_stick_x);
            LBpower = LBpower + (gamepad1.right_stick_x);

            if (gamepad2.left_trigger > 0.2){
                sweeperPower = -gamepad2.left_trigger;
            }
            if (gamepad2.right_trigger > 0.2){
                sweeperPower = gamepad2.right_trigger;
            }

            if (gamepad1.y){
                geleiderPower = 1;
            }
            if (gamepad1.a) {
                geleiderPower = -0.5;
            }


            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);
            LFdrive.setPower(LFpower * fastency);
            RBdrive.setPower(RBpower * fastency);
            LBdrive.setPower(LBpower * fastency);
            RFdrive.setPower(RFpower * fastency);
            geleider.setPower(geleiderPower);
            sweeper.setPower(sweeperPower);
        }
    }
}


