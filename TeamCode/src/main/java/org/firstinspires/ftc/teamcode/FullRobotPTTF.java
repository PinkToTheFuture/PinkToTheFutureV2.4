package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
        int shooterPosition = 0;

        Servo Armservo = hardwareMap.servo.get("servoarm");
        Armservo.setPosition(0.5);
        Servo Armrelease1 = hardwareMap.servo.get("servoarmrelease1");
        Servo Armrelease2 = hardwareMap.servo.get("servoarmrelease2");
        Armrelease1.setPosition(0);
        Armrelease2.setPosition(0.9);
        Servo shooterservoX = hardwareMap.servo.get("shooterservox");
        shooterservoX.setPosition(0.5);


        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        DcMotor sweeper = hardwareMap.dcMotor.get("sweeper");
        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        DcMotor geleider1 = hardwareMap.dcMotor.get("geleider1");
        DcMotor geleider2 = hardwareMap.dcMotor.get("geleider2");


        TouchSensor shootertouch = hardwareMap.touchSensor.get("shootertouch");

        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        geleider1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setPower(-1);



        waitOneFullHardwareCycle();
        waitForStart();
        while (opModeIsActive()) {
            waitOneFullHardwareCycle();
            if (gamepad1.dpad_up)     fastency = 1;
            if (gamepad1.dpad_down)   fastency = 0.3;
            RFpower = 0;
            RBpower = 0;
            LFpower = 0;
            LBpower = 0;
            geleiderPower = 0;
            sweeperPower = 0;


            RFpower = -((gamepad1.left_stick_y + gamepad1.left_stick_x) / 2);
            RBpower = -((gamepad1.left_stick_y - gamepad1.left_stick_x) / 2);
            LFpower = -((gamepad1.left_stick_y - gamepad1.left_stick_x) / 2);
            LBpower = -((gamepad1.left_stick_y + gamepad1.left_stick_x) / 2);



         /**   if (gamepad1.left_stick_x >= 0 && gamepad1.left_stick_y < 0){
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
          **/

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

            if (gamepad2.back) {
                Armrelease1.setPosition(0.3);
                Armrelease2.setPosition(0.7);
            }



            if (gamepad2.b) {
                Armservo.setPosition(1);
            } else {
                if (gamepad2.x){
                    Armservo.setPosition(0);
                } else {
                    Armservo.setPosition(0.5);
                }
            }

            if (gamepad1.right_bumper) {
                shooterPosition = shooterPosition + 2240;
                shooter.setTargetPosition(shooterPosition);
                while (gamepad1.right_bumper){
                    idle();
                }
            }

            if (gamepad1.dpad_up) {
                shooterPosition = shooterPosition+20;
                shooter.setTargetPosition(shooterPosition);
                while (gamepad1.dpad_up){
                    idle();
                }
            }
            if (gamepad1.dpad_down){
                shooterPosition = shooterPosition-20;
                shooter.setTargetPosition(shooterPosition);
                while (gamepad1.dpad_down){
                    idle();
                }
            }



            if (gamepad2.y){
                geleiderPower = 1;
            }
            if (gamepad2.a) {
                geleiderPower = -0.3;
            }


            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);
            LFdrive.setPower(LFpower * fastency);
            RBdrive.setPower(RBpower * fastency);
            LBdrive.setPower(LBpower * fastency);
            RFdrive.setPower(RFpower * fastency);
            geleider1.setPower(geleiderPower);
            geleider2.setPower(geleiderPower);
            sweeper.setPower(sweeperPower);


            telemetry.addData("shooter encoder", shooter.getCurrentPosition());
            telemetry.addData("shootertouch", shootertouch);
            telemetry.addData("shooterPosition", shooterPosition);
            telemetry.update();

        }
        shooter.setPower(0);
    }
}
