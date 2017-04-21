package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Cali Sensor", group = "PinktotheFuture")

public class CaliSensors extends LinearOpMode {

    @Override public void runOpMode() throws InterruptedException {
        ColorSensor Lcolor = hardwareMap.colorSensor.get("lcolor");
        Lcolor.setI2cAddress(I2cAddr.create8bit(0x3c));
        Lcolor.enableLed(false);
        ColorSensor Rcolor = hardwareMap.colorSensor.get("rcolor");
        Rcolor.setI2cAddress(I2cAddr.create8bit(0x2c));
        Rcolor.enableLed(false);



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


        ModernRoboticsI2cGyro gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro.setI2cAddress(I2cAddr.create8bit(0x20));

        LightSensor Flight = hardwareMap.lightSensor.get("Flight");
        LightSensor Blight = hardwareMap.lightSensor.get("Blight");
        Flight.enableLed(true);
        Blight.enableLed(true);


        gyro.calibrate();
        while (gyro.isCalibrating()){
            idle();
        }

        waitForStart();

        while (opModeIsActive()) {
            Rrangesensorcache = Rrangereader.read(0x04, 2);
            int Rrange = Rrangesensorcache[0] & 0xFF;

            Lrangesensorcache = Lrangereader.read(0x04, 2);
            int Lrange = Lrangesensorcache[0] & 0xFF;
            telemetry.addData("Rultrasonic", Rrange);
            telemetry.addData("Lultrasonic", Lrange);
            telemetry.addData("gyro", gyro.getHeading());
            telemetry.addData("Llight raw", Flight.getRawLightDetected());
            telemetry.addData("Rlight deze", Blight.getRawLightDetected());
            telemetry.addData("Lcolor blue", Lcolor.blue());
            telemetry.addData("Lcolor red", Lcolor.red());
            telemetry.addData("Rcolor blue", Rcolor.blue());
            telemetry.addData("Rcolor red", Rcolor.red());
            telemetry.update();
            idle();
        }


    }
}
