package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Andrew on 11/10/2017.
 */
@TeleOp(name="Autonomous", group="TeleOp")

public class Autonomous extends OpMode {
    DcMotor rmotor0; //0 is the front
    DcMotor lmotor0;
    DcMotor rmotor1;
    DcMotor lmotor1;
    Servo jewelKnocker;
    ColorSensor colorSensor;
    enum state  {dropArm,turn,stop}
    state state;
    public void moveMotors(double lMotor0, double lMotor1,double rMotor0, double rMotor1){

    }

    public void init (){

        lmotor0 = hardwareMap.dcMotor.get("lmotor0");
        rmotor0 = hardwareMap.dcMotor.get("rmotor0");
        lmotor1 = hardwareMap.dcMotor.get("lmotor1");
        rmotor1 = hardwareMap.dcMotor.get("rmotor1");
        lmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rmotor0.setDirection(DcMotor.Direction.REVERSE);
        rmotor1.setDirection(DcMotor.Direction.REVERSE);
        colorSensor =  hardwareMap.colorSensor.get("colorSensor");
        lmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        jewelKnocker = hardwareMap.servo.get("jewelKnocker");
        lmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        state=state.dropArm;


    }
    public void loop(){

        switch (state) {

            case dropArm:
                jewelKnocker.setPosition(1);
                try {
                    Thread.sleep(1000);
                }
                catch (InterruptedException e) {
                    e.printStackTrace();
                }
                state=state.turn;
            break;
            case turn:
                if (colorSensor.red() < colorSensor.blue()) {
                    lmotor0.setTargetPosition(500);
                    lmotor1.setTargetPosition(500);
                    lmotor0.setPower(1);
                    lmotor1.setPower(1);

                }
                else if (colorSensor.blue() < colorSensor.red()) {

                    rmotor0.setTargetPosition(500);
                    rmotor1.setTargetPosition(500);
                    rmotor0.setPower(1);
                    rmotor1.setPower(1);
                    state=state.stop;
                }
            break;
            case stop:
                telemetry.addData("done","done");
                break;

        }

    }
}