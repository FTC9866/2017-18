package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="TeleOp", group="TeleOp")

public class Drive extends VirusMethods {
    public void start(){
        lmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void loop(){
        updateControllerValues();
        if (leftx!=0 || lefty!=0){
            runMotors(-var1,-var2,-var2,-var1,rightx);
        } else {
            runMotors(0,0,0,0,rightx);
        }
        if (gamepad1.right_bumper){
            maxPower=1;
        }
        else if (gamepad1.left_bumper){
            maxPower=.4;
        }
        if (gamepad2.y) {
            cube1.setPosition(0.0);
            cube2.setPosition(1);
        }
        if (gamepad2.x) {
            cube1.setPosition(.5);
            cube2.setPosition(.5);
        }
        telemetry.addData("Red",colorSensor.red());
        telemetry.addData("Green",colorSensor.green());
        telemetry.addData("Blue",colorSensor.blue());
    }
}