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
            runMotors(var1,var2,var2,var1,rightx); //var1 and 2 are computed values found in theUpdateControllerValues method
        } else {
            runMotors(0,0,0,0,rightx);
        }
        if (gamepad1.right_bumper){
            maxPower=1;
        }
        else if (gamepad1.left_bumper){
            maxPower=.4;
        }
        if (gamepad2.left_bumper) {
            cube1.setPosition(0.0);
            cube2.setPosition(1);
        }
        if (gamepad2.right_bumper) {
            cube1.setPosition(.6);
            cube2.setPosition(.4);
     }
        if (gamepad2.left_trigger>0.5){
            cube3.setPosition(.15);
            cube4.setPosition(.85);
        }
        if (gamepad2.right_trigger>0.5){
            cube3.setPosition(.6);
            cube4.setPosition(.4);
        }
        if (gamepad2.a){
            lift.setPosition(0);
        }
        if (gamepad2.b){
            lift.setPosition(.3/3 + .02);
        }
        if (gamepad2.y){
            lift.setPosition(.6/3 + .04);
        }
        if (gamepad2.x){
            lift.setPosition(.3 + .02);
        }

       // Telemetry();
    }
}
