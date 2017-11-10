package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Andrew on 9/22/2017.
 */

@TeleOp(name="TeleOp", group="TeleOp")

public class Drive extends OpMode {
    DcMotor rmotor0; //0 is the front
    DcMotor lmotor0;
    DcMotor rmotor1;
    DcMotor lmotor1;
    ColorSensor colorSensor;
    double maxPower = 1;
    double steerMagnitude=0;
    public void runMotors(double Left0, double Left1, double Right0, double Right1){
        if (Left0!=0&&Left1!=0&&Right0!=0&&Right1!=0) {
            steerMagnitude *= 2 * -Math.max(Math.max(Left0, Left1), Math.max(Right0, Right1));
        }
        Left0=Left0+steerMagnitude;
        Left1=Left1+steerMagnitude;
        Right0=Right0-steerMagnitude;
        Right1=Right1-steerMagnitude;
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
    public void init(){
        lmotor0 = hardwareMap.dcMotor.get("lmotor0");
        rmotor0 = hardwareMap.dcMotor.get("rmotor0");
        lmotor1 = hardwareMap.dcMotor.get("lmotor1");
        rmotor1 = hardwareMap.dcMotor.get("rmotor1");
        lmotor0.setDirection(DcMotor.Direction.REVERSE);
        lmotor1.setDirection(DcMotor.Direction.REVERSE);
        lmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo cube1;
        Servo cube2;
        colorSensor = hardwareMap.colorSensor.get("colorsensor");
    }
    public void loop(){

        double lefty = -(gamepad1.left_stick_y);
        double leftx = -gamepad1.left_stick_x;
        double righty = -gamepad1.right_stick_y;
        double rightx = -gamepad1.right_stick_x;
        double rtrigger = -gamepad1.right_trigger;
        double ltrigger = -gamepad1.left_trigger;
        double var1= (lefty-leftx)/Math.sqrt(2);
        double var2= (lefty+leftx)/Math.sqrt(2);
        steerMagnitude=rightx;
        if (leftx == 0 && lefty==0) {
            runMotors(0,0,0,0);
        }
        if (leftx!=0 || lefty!=0){
            runMotors(-var1,-var2,-var2,-var1);
        }
        if (gamepad1.right_bumper){
            maxPower=1;
        }
        else if (gamepad1.left_bumper){
            maxPower=.4;
        }
        if(){

        }
        telemetry.addData("Red",colorSensor.red());
        telemetry.addData("Green",colorSensor.green());
        telemetry.addData("Blue",colorSensor.blue());
    }
}
