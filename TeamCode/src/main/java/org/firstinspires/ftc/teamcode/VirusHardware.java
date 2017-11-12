package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by mzhang on 11/12/2017.
 */

public abstract class VirusHardware extends OpMode {
    DcMotor rmotor0; //0 is the front
    DcMotor lmotor0;
    DcMotor rmotor1;
    DcMotor lmotor1;
    final double wheelDiam=4.0;
    final double gearRatio=1.5;
    final double pulsePerRotation=280.0;
    final double inPerWheelRotation=wheelDiam * Math.PI;
    final double inPerMotorRotation=inPerWheelRotation*gearRatio;
    final double inPerPulse=inPerMotorRotation/pulsePerRotation;
    Servo jewelKnocker;
    ColorSensor colorSensor;
    Servo cube1;
    Servo cube2;
    double maxPower=1;
    double lefty;
    double leftx;
    double righty;
    double rightx;
    double rtrigger;
    double ltrigger;
    double var1;
    double var2;
    public void init(){
        lmotor0 = hardwareMap.dcMotor.get("lmotor0");
        rmotor0 = hardwareMap.dcMotor.get("rmotor0");
        lmotor1 = hardwareMap.dcMotor.get("lmotor1");
        rmotor1 = hardwareMap.dcMotor.get("rmotor1");
        rmotor0.setDirection(DcMotor.Direction.REVERSE);
        rmotor1.setDirection(DcMotor.Direction.REVERSE);
        lmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cube1 = hardwareMap.servo.get("cube1");
        cube2 = hardwareMap.servo.get("cube2");
        jewelKnocker = hardwareMap.servo.get("jewelKnocker");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        cube1.setPosition(0);
        cube2.setPosition(1);
        jewelKnocker.setPosition(0);
    }

}