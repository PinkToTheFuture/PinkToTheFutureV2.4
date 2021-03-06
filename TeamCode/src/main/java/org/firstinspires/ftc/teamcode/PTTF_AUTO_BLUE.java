package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import static org.firstinspires.ftc.teamcode.RobotVariables.ArmservoStopPosition;
import static org.firstinspires.ftc.teamcode.RobotVariables.releaseArmLStartPosition;
import static org.firstinspires.ftc.teamcode.RobotVariables.releaseArmRStartPosition;
import static org.firstinspires.ftc.teamcode.RobotVariables.shooterservoXStartPosition;

@Autonomous(name = "AUTO BLUE", group = "full")

public class PTTF_AUTO_BLUE extends LinearOpMode {
    private void Forward(double omw, double pwr) throws InterruptedException{
        boolean loop = true;
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        LBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        idle();

        LFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        LFdrive.setTargetPosition((int) (omw * 11.20));
        LBdrive.setTargetPosition((int) (omw * 11.20));
        RFdrive.setTargetPosition((int) (omw * 11.20));
        RBdrive.setTargetPosition((int) (omw * 11.20));

        int marge = (int)((omw * 11.20) - 40);

        while (loop && opModeIsActive()){
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (LFdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 1.50);
                LBdrive.setPower(pwr * 1.50);
                RFdrive.setPower(pwr * 0.66);
                RBdrive.setPower(pwr * 0.66);
            }
            if (LFdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 0.66);
                LBdrive.setPower(pwr * 0.66);
                RFdrive.setPower(pwr * 1.50);
                RBdrive.setPower(pwr * 1.50);
            }
            if (LFdrive.getCurrentPosition() > marge && LBdrive.getCurrentPosition() > marge && RFdrive.getCurrentPosition() > marge && RBdrive.getCurrentPosition() > marge){
                loop = false;
                telemetry.addData("loop is false", "");
                telemetry.update();
                sleep(500);
            }
        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);

    }
    private void Reverse(double omw, double pwr) throws InterruptedException{
        boolean loop = true;
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        idle();

        LFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        LFdrive.setTargetPosition((int) omw * 1120);
        LBdrive.setTargetPosition((int) omw * 1120);
        RFdrive.setTargetPosition((int) omw * 1120);
        RBdrive.setTargetPosition((int) omw * 1120);

        while (loop && opModeIsActive()){
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (LFdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 0.75);
                LBdrive.setPower(pwr * 0.75);
                RFdrive.setPower(pwr * 1.33);
                RBdrive.setPower(pwr * 1.33);
            }
            if (LFdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 1.33);
                LBdrive.setPower(pwr * 1.33);
                RFdrive.setPower(pwr * 0.75);
                RBdrive.setPower(pwr * 0.75);
            }
            if (LFdrive.getCurrentPosition() > (omw*11.20 - 40) && LBdrive.getCurrentPosition() > (omw*11.20 - 40) && RFdrive.getCurrentPosition() > (omw*11.20 - 40) && RBdrive.getCurrentPosition() > (omw*11.20 - 40)) {
                loop = false;
            }
        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    private void Right_Gyro(double degrees, double pwr, double sloommultiplier) throws InterruptedException{
        ModernRoboticsI2cGyro gyro = hardwareMap.get((ModernRoboticsI2cGyro.class), "gyro");


        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeIsActive() && gyro.getHeading() < (degrees - 10) || gyro.getHeading() > 340){
            LFdrive.setPower(pwr);
            LBdrive.setPower(pwr);
            RFdrive.setPower(-pwr);
            RBdrive.setPower(-pwr);
            idle();
            telemetry.addData("gyro", gyro.getHeading());
            telemetry.update();

        }
        boolean loop = true;
        while (opModeIsActive() && loop){
            while (opModeIsActive() && !(gyro.getHeading() == degrees)){
                if (gyro.getHeading() > degrees) {
                    LFdrive.setPower(-pwr * sloommultiplier);
                    LBdrive.setPower(-pwr * sloommultiplier);
                    RFdrive.setPower(pwr * sloommultiplier);
                    RBdrive.setPower(pwr * sloommultiplier);
                }
                if (gyro.getHeading() < degrees) {
                    LFdrive.setPower(pwr * sloommultiplier);
                    LBdrive.setPower(pwr * sloommultiplier);
                    RFdrive.setPower(-pwr * sloommultiplier);
                    RBdrive.setPower(-pwr * sloommultiplier);
                }
                telemetry.addData("gyro", gyro.getHeading());
                telemetry.update();
            }
            LFdrive.setPower(0);
            LBdrive.setPower(0);
            RFdrive.setPower(0);
            RBdrive.setPower(0);
            sleep(50);
            if (gyro.getHeading() == degrees){
                loop = false;
            }
        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);

        }

    private void Left_Gyro(double degrees, double pwr, double sloommultiplier) throws InterruptedException{
        ModernRoboticsI2cGyro gyro = hardwareMap.get((ModernRoboticsI2cGyro.class), "gyro");

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeIsActive() && (gyro.getHeading() < 180 || gyro.getHeading() > (degrees + 35))){
            RFdrive.setPower(pwr);
            RBdrive.setPower(pwr);
            LFdrive.setPower(-pwr);
            LBdrive.setPower(-pwr);
            idle();
            telemetry.addData("gyro", gyro.getHeading());
            telemetry.update();

        }

        while (opModeIsActive() && !(gyro.getHeading() == (degrees))){
            if (gyro.getHeading() < degrees) {
                RFdrive.setPower(-pwr * sloommultiplier);
                RBdrive.setPower(-pwr * sloommultiplier);
                LFdrive.setPower(pwr * sloommultiplier);
                LBdrive.setPower(pwr * sloommultiplier);
            }
            if (gyro.getHeading() > degrees) {
                LFdrive.setPower(-pwr * sloommultiplier);
                LBdrive.setPower(-pwr * sloommultiplier);
                RFdrive.setPower(pwr * sloommultiplier);
                RBdrive.setPower(pwr * sloommultiplier);
            }
            telemetry.addData("gyro", gyro.getHeading());
            telemetry.update();

        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    private void init_gyro() throws InterruptedException{
        ModernRoboticsI2cGyro gyro = hardwareMap.get((ModernRoboticsI2cGyro.class), "gyro");
        gyro.calibrate();
        while (gyro.isCalibrating()){
            idle();
        }
        telemetry.addData("gyro calibrated", gyro.status());
        telemetry.update();
    }

    private void shoot() throws InterruptedException{
        Servo shooterservoX = hardwareMap.servo.get("shooterservox");
        TouchSensor shootertouch = hardwareMap.touchSensor.get("shootertouch");

        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setPower(1);
        shooter.setTargetPosition(2240);

        boolean loop = true;
        while (loop && opModeIsActive()){
            if (shooter.getCurrentPosition() > 2220){
                loop = false;
            }
        }

        shooterservoX.setPosition(0.15);
        sleep(200);
        while (opModeIsActive() && !shootertouch.isPressed()){
            idle();
        }
        shooterservoX.setPosition(0.5);

        shooter.setTargetPosition(4480);

        loop = true;
        while (loop && opModeIsActive()){
            if (shooter.getCurrentPosition() > 4200){
                loop = false;
            }
        }
    }

    private void Left_Sideways(double omw, double pwr) throws InterruptedException {
        boolean loop = true;
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        idle();

        LFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        LFdrive.setTargetPosition((int) (omw * 11.20));
        LBdrive.setTargetPosition((int) (omw * 11.20));
        RFdrive.setTargetPosition((int) (omw * 11.20));
        RBdrive.setTargetPosition((int) (omw * 11.20));

        while (loop && opModeIsActive()){
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (LFdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 0.75);
            }
            if (LFdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 1.33);
            }


            if (LBdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LBdrive.setPower(pwr * 0.75);
            }
            if (LBdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                LBdrive.setPower(pwr * 1.33);
            }


            if (RBdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                RBdrive.setPower(pwr * 0.75);
            }
            if (RBdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                RBdrive.setPower(pwr * 1.33);
            }


            if (LFdrive.getCurrentPosition() > (omw*11.20 - 40) && LBdrive.getCurrentPosition() > (omw*11.20 - 40) && RFdrive.getCurrentPosition() > (omw*11.20 - 40) && RBdrive.getCurrentPosition() > (omw*11.20 - 40)) {
                loop = false;
            }
        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    private void Right_Sideways(double omw, double pwr) throws InterruptedException {
        boolean loop = true;
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        idle();

        LFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        LFdrive.setTargetPosition((int) (omw * 11.20));
        LBdrive.setTargetPosition((int) (omw * 11.20));
        RFdrive.setTargetPosition((int) (omw * 11.20));
        RBdrive.setTargetPosition((int) (omw * 11.20));

        while (loop && opModeIsActive()){
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (LFdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 0.75);
            }
            if (LFdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                LFdrive.setPower(pwr * 1.33);
            }


            if (LBdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                LBdrive.setPower(pwr * 0.75);
            }
            if (LBdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                LBdrive.setPower(pwr * 1.33);
            }


            if (RBdrive.getCurrentPosition() > RFdrive.getCurrentPosition()){
                RBdrive.setPower(pwr * 0.75);
            }
            if (RBdrive.getCurrentPosition() < RFdrive.getCurrentPosition()){
                RBdrive.setPower(pwr * 1.33);
            }


            if (LFdrive.getCurrentPosition() > (omw*11.20 - 40) && LBdrive.getCurrentPosition() > (omw*11.20 - 40) && RFdrive.getCurrentPosition() > (omw*11.20 - 40) && RBdrive.getCurrentPosition() > (omw*11.20 - 40)) {
                loop = false;
            }
        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    private void FollowRightLine(double pwr, double Threshold) throws InterruptedException{
        double FollowPowerFront = 0;
        double FollowPowerTurn = 0;

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LightSensor Flight = hardwareMap.lightSensor.get("Flight");
        LightSensor Blight = hardwareMap.lightSensor.get("Blight");
        Flight.enableLed(true);
        Blight.enableLed(true);

        byte[] Lrangesensorcache;
        I2cDevice LrangeSensor = hardwareMap.i2cDevice.get("lrangesensor");
        I2cDeviceSynch Lrangereader;
        Lrangereader = new I2cDeviceSynchImpl(LrangeSensor, I2cAddr.create8bit(0x10), false);
        Lrangereader.engage();


        byte[] Rrangesensorcache;
        I2cDevice RrangeSensor = hardwareMap.i2cDevice.get("rrangesensor");
        I2cDeviceSynch Rrangereader;
        Rrangereader = new I2cDeviceSynchImpl(RrangeSensor, I2cAddr.create8bit(0x28), false);
        Rrangereader.engage();


        Lrangesensorcache = Lrangereader.read(0x04, 2);
        int Lrange = Lrangesensorcache[0] & 0xFF;
        while (Lrange > 12 && opModeIsActive()){
            Lrangesensorcache = Lrangereader.read(0x04, 2);
            Lrange = Lrangesensorcache[0] & 0xFF;

            FollowPowerFront = 0;
            FollowPowerTurn = 0;
            if (Flight.getRawLightDetected() > Threshold) {
                FollowPowerFront = -0.12;
                telemetry.addData("Flight = white", Flight.getRawLightDetected());
            }
            if (Flight.getRawLightDetected() < Threshold) {
                FollowPowerFront = 0.1;
                telemetry.addData("Flight = black", Flight.getRawLightDetected());
            }

            if (Blight.getRawLightDetected() > Threshold) {
                FollowPowerTurn = -0.05;
                telemetry.addData("Blight = white", Blight.getRawLightDetected());
            }
            if (Blight.getRawLightDetected() < Threshold) {
                FollowPowerTurn = 0.05;
                telemetry.addData("Blight = black", Blight.getRawLightDetected());
            }


            //FollowPowerFront = 0;
            //FollowPowerTurn = 0;
            LFdrive.setPower(pwr + (-FollowPowerTurn + FollowPowerFront));
            LBdrive.setPower(pwr + (-FollowPowerTurn - FollowPowerFront));
            RFdrive.setPower(pwr + (FollowPowerTurn - FollowPowerFront));
            RBdrive.setPower(pwr + (FollowPowerTurn + FollowPowerFront));

            telemetry.addData("rangesensor:", Lrange);
            telemetry.update();

        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);

    }


    private void FollowLeftLine(double pwr, double Threshold) throws InterruptedException{
        double FollowPowerFront;
        double FollowPowerTurn;

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LightSensor Flight = hardwareMap.lightSensor.get("Flight");
        LightSensor Blight = hardwareMap.lightSensor.get("Blight");
        Flight.enableLed(true);
        Blight.enableLed(true);

        byte[] Lrangesensorcache;
        I2cDevice LrangeSensor = hardwareMap.i2cDevice.get("lrangesensor");
        I2cDeviceSynch Lrangereader;
        Lrangereader = new I2cDeviceSynchImpl(LrangeSensor, I2cAddr.create8bit(0x10), false);
        Lrangereader.engage();


        byte[] Rrangesensorcache;
        I2cDevice RrangeSensor = hardwareMap.i2cDevice.get("rrangesensor");
        I2cDeviceSynch Rrangereader;
        Rrangereader = new I2cDeviceSynchImpl(RrangeSensor, I2cAddr.create8bit(0x28), false);
        Rrangereader.engage();


        Lrangesensorcache = Lrangereader.read(0x04, 2);
        int Lrange = Lrangesensorcache[0] & 0xFF;
        while (Lrange > 12 && opModeIsActive()){
            Lrangesensorcache = Lrangereader.read(0x04, 2);
            Lrange = Lrangesensorcache[0] & 0xFF;

            FollowPowerFront = 0;
            FollowPowerTurn = 0;
            if (Flight.getRawLightDetected() > Threshold) {
                FollowPowerFront = -0.12;
                telemetry.addData("Flight = white", Flight.getRawLightDetected());
            }
            if (Flight.getRawLightDetected() < Threshold) {
                FollowPowerFront = 0.1;
                telemetry.addData("Flight = black", Flight.getRawLightDetected());
            }

            if (Blight.getRawLightDetected() > Threshold) {
                FollowPowerTurn = -0.05;
                telemetry.addData("Blight = white", Blight.getRawLightDetected());
            }
            if (Blight.getRawLightDetected() < Threshold) {
                FollowPowerTurn = 0.05;
                telemetry.addData("Blight = black", Blight.getRawLightDetected());
            }


            //FollowPowerFront = 0;
            //FollowPowerTurn = 0;
            LFdrive.setPower(pwr + (-FollowPowerTurn + FollowPowerFront));
            LBdrive.setPower(pwr + (-FollowPowerTurn - FollowPowerFront));
            RFdrive.setPower(pwr + (FollowPowerTurn - FollowPowerFront));
            RBdrive.setPower(pwr + (FollowPowerTurn + FollowPowerFront));

            telemetry.addData("rangesensor:", Lrange);
            telemetry.update();

        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);

    }
    private void Push() throws InterruptedException{
        ColorSensor Lcolor = hardwareMap.colorSensor.get("lcolor");
        Lcolor.setI2cAddress(I2cAddr.create8bit(0x2c));
        ColorSensor Rcolor = hardwareMap.colorSensor.get("rcolor");
        Rcolor.setI2cAddress(I2cAddr.create8bit(0x3c));
        sleep(50);
        boolean dosome = true;
        while (dosome && opModeIsActive()) {
            if (Rcolor.red() < Rcolor.blue()) {
                telemetry.addData("red > blue", "");
                Left_Sideways(80, 0.3);
                Reverse(30, 0.2);
                dosome = false;
            }
            if (Rcolor.red() > Rcolor.blue() && dosome) {
                telemetry.addData("red < blue", "");
                Right_Sideways(100, 0.3);
                Reverse(30, 0.2);
                dosome = false;
            }
            telemetry.addData("r red", Rcolor.red());
            telemetry.addData("r blue", Rcolor.blue());
            telemetry.addData("l red", Lcolor.red());
            telemetry.addData("l blue", Lcolor.blue());
            telemetry.update();
        }
    }

    private void FollowWallLeft(double omw, double pwr, double afstand, double threshold) throws InterruptedException{
        ModernRoboticsI2cGyro gyro = hardwareMap.get((ModernRoboticsI2cGyro.class), "gyro");

        byte[] Lrangesensorcache;
        I2cDevice LrangeSensor = hardwareMap.i2cDevice.get("lrangesensor");
        I2cDeviceSynch Lrangereader;
        Lrangereader = new I2cDeviceSynchImpl(LrangeSensor, I2cAddr.create8bit(0x10), false);
        Lrangereader.engage();


        byte[] Rrangesensorcache;
        I2cDevice RrangeSensor = hardwareMap.i2cDevice.get("rrangesensor");
        I2cDeviceSynch Rrangereader;
        Rrangereader = new I2cDeviceSynchImpl(RrangeSensor, I2cAddr.create8bit(0x28), false);
        Rrangereader.engage();


        LightSensor Flight = hardwareMap.lightSensor.get("Flight");
        LightSensor Blight = hardwareMap.lightSensor.get("Blight");
        Flight.enableLed(true);
        Blight.enableLed(true);


        double FollowPowerFront = 0;
        double FollowPowerTurn = 0;
        boolean loop = true;

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        LBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        idle();

        LFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        while (loop && opModeIsActive()){
            Rrangesensorcache = Rrangereader.read(0x04, 2);
            int Rrange = Rrangesensorcache[0] & 0xFF;

            Lrangesensorcache = Lrangereader.read(0x04, 2);
            int Lrange = Lrangesensorcache[0] & 0xFF;
            telemetry.addData("Rultrasonic", Rrange);
            telemetry.addData("Lultrasonic", Lrange);
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (Lrange > afstand) FollowPowerFront = -0.1;
            if (Lrange < afstand) FollowPowerFront = 0.1;

            double tmp = gyro.getHeading();
            if (tmp > 270) FollowPowerTurn = -0.15;
            if (tmp < 270) FollowPowerTurn = 0.15;
            if (tmp == 270) FollowPowerTurn = 0;

            //FollowPowerFront = 0;
            //FollowPowerTurn = 0;

            if (LFdrive.getCurrentPosition() > omw * 11.20){
                loop = false;
                telemetry.addData("loop is false", "");
                telemetry.update();
            }

            LFdrive.setPower(pwr + (FollowPowerFront - FollowPowerTurn));
            LBdrive.setPower(-pwr + (FollowPowerFront - FollowPowerTurn));
            RFdrive.setPower(-pwr + (FollowPowerFront + FollowPowerTurn));
            RBdrive.setPower(pwr + (FollowPowerFront + FollowPowerTurn));
        }

        while (Flight.getRawLightDetected() < threshold && Blight.getRawLightDetected() < threshold && opModeIsActive()){
            Rrangesensorcache = Rrangereader.read(0x04, 2);
            int Rrange = Rrangesensorcache[0] & 0xFF;

            Lrangesensorcache = Lrangereader.read(0x04, 2);
            int Lrange = Lrangesensorcache[0] & 0xFF;
            telemetry.addData("Rultrasonic", Rrange);
            telemetry.addData("Lultrasonic", Lrange);
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (Lrange > afstand ) FollowPowerFront = -0.1;
            if (Lrange < afstand ) FollowPowerFront = 0.1;

            if (Lrange > Rrange) FollowPowerTurn = -0.09;
            if (Lrange < Rrange) FollowPowerTurn = 0.09;
            if (Lrange == Rrange) FollowPowerTurn = 0;

            double pwrmultiplier = 0.2;
            LFdrive.setPower(pwr * pwrmultiplier + (FollowPowerFront - FollowPowerTurn));
            LBdrive.setPower(-pwr * pwrmultiplier + (FollowPowerFront - FollowPowerTurn));
            RFdrive.setPower(-pwr * pwrmultiplier + (FollowPowerFront + FollowPowerTurn));
            RBdrive.setPower(pwr * pwrmultiplier + (FollowPowerFront + FollowPowerTurn));
        }


        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);

    }
    private void DriveToLineRight(double pwr, double threshold) throws InterruptedException{
        boolean loop = true;

        LightSensor Flight = hardwareMap.lightSensor.get("Flight");
        LightSensor Blight = hardwareMap.lightSensor.get("Blight");
        Flight.enableLed(true);
        Blight.enableLed(true);

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        idle();

        LFdrive.setPower(pwr);
        LBdrive.setPower(pwr);
        RFdrive.setPower(pwr);
        RBdrive.setPower(pwr);

        while (loop && opModeIsActive()){
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (Blight.getRawLightDetected() > threshold | Flight.getRawLightDetected() > threshold){
                loop = false;
            }
        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    @Override public void runOpMode() throws InterruptedException {


        Servo Armservo = hardwareMap.servo.get("servoarm");
        Armservo.setPosition(ArmservoStopPosition);

        Servo releaseArmL = hardwareMap.servo.get("releasearmL");
        Servo releaseArmR = hardwareMap.servo.get("releasearmR");
        releaseArmL.setPosition(releaseArmLStartPosition);
        releaseArmR.setPosition(releaseArmRStartPosition);
        Servo shooterservoX = hardwareMap.servo.get("shooterservox");
        shooterservoX.setPosition(shooterservoXStartPosition);


        double threshold = 1.9;


        init_gyro();
        waitForStart();

        Forward(130, 0.4);
        shoot();
        Right_Gyro(46, 0.18, 0.61);
        Forward(483, 0.5);
        Left_Gyro(268, 0.29, 0.47);

        DriveToLineRight(0.28, threshold);

        FollowLeftLine(0.09, threshold);

        Push();

        FollowWallLeft(420, 0.6, 15, threshold);
        Push();
    }
}
