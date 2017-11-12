package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by mzhang on 11/12/2017.
 */

public class VirusAuto extends OpMode{
    DcMotor rmotor0; //0 is the front
    DcMotor lmotor0;
    DcMotor rmotor1;
    DcMotor lmotor1;
    final double wheelDiam=4.0;
    final double gearRatio=1.5;
    final double pulsePerRotaton=280.0;
    final double inPerWheelRotation=wheelDiam * Math.PI;
    final double inPerMotorRotation=inPerWheelRotation*1.5;
    final double inPerPulse=inPerMotorRotation/280;
    Servo jewelKnocker;
    ColorSensor colorSensor;
    enum state  {dropArm,knockJewel,stop}
    state state;
    Servo cube1;
    Servo cube2;
    double maxPower=1;
    public void runMotors(double Left0, double Left1, double Right0, double Right1, double steerMagnitude){
        if (Left0!=0&&Left1!=0&&Right0!=0&&Right1!=0) {
            steerMagnitude *= -2 * Math.max(Math.max(Left0, Left1), Math.max(Right0, Right1));
        }
        Left0=Left0-steerMagnitude;
        Left1=Left1-steerMagnitude;
        Right0=Right0+steerMagnitude;
        Right1=Right1+steerMagnitude;
        //make sure no exception thrown if power > 0
        Left0 = Range.clip(Left0, -maxPower, maxPower);
        Left1 = Range.clip(Left1, -maxPower, maxPower);
        Right0 = Range.clip(Right0, -maxPower, maxPower);
        Right1 = Range.clip(Right1, -maxPower, maxPower);
        rmotor0.setPower(Right0);
        rmotor1.setPower(Right1);
        lmotor0.setPower(Left0);
        lmotor1.setPower(Left1);
    }
    public void runMotors(double Left0, double Left1, double Right0, double Right1){
        //make sure no exception thrown if power > 0
        Left0 = Range.clip(Left0, -maxPower, maxPower);
        Left1 = Range.clip(Left1, -maxPower, maxPower);
        Right0 = Range.clip(Right0, -maxPower, maxPower);
        Right1 = Range.clip(Right1, -maxPower, maxPower);
        rmotor0.setPower(Right0);
        rmotor1.setPower(Right1);
        lmotor0.setPower(Left0);
        lmotor1.setPower(Left1);
    }
    public void setMotorPositions(int Left0, int Left1, int Right0, int Right1, double power){
        resetEncoder();
        lmotor0.setTargetPosition(Left0);
        lmotor1.setTargetPosition(Left1);
        rmotor0.setTargetPosition(Right0);
        rmotor1.setTargetPosition(Right1);
        runMotors(power,power,power,power);
    }
    public void setMotorPositionsINCH(double Left0, double Left1, double Right0, double Right1, double power){
        resetEncoder();
        lmotor0.setTargetPosition((int)(Left0/inPerPulse));
        lmotor1.setTargetPosition((int)(Left1/inPerPulse));
        rmotor0.setTargetPosition((int)(Right0/inPerPulse));
        rmotor1.setTargetPosition((int)(Right1/inPerPulse));
        runMotors(power,power,power,power);
    }
    public void resetEncoder(){
        runMotors(0,0,0,0);
        lmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (lmotor0.isBusy()||lmotor1.isBusy()||rmotor0.isBusy()||rmotor1.isBusy());
    }
    public void init() {
        lmotor0 = hardwareMap.dcMotor.get("lmotor0");
        rmotor0 = hardwareMap.dcMotor.get("rmotor0");
        lmotor1 = hardwareMap.dcMotor.get("lmotor1");
        rmotor1 = hardwareMap.dcMotor.get("rmotor1");
        resetEncoder();
        rmotor0.setDirection(DcMotor.Direction.REVERSE);
        rmotor1.setDirection(DcMotor.Direction.REVERSE);
        lmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cube1 = hardwareMap.servo.get("cube1");
        cube2 = hardwareMap.servo.get("cube2");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        cube1.setPosition(0);
        cube2.setPosition(1);
    }


    public void loop() {

    }


}
