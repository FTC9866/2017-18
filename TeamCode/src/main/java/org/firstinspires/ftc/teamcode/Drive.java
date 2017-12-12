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
            cube1.setPosition(.5);
            cube2.setPosition(.5);
     }
     /*   if (gamepad2.left_trigger>0.5){
            cube3.setPosition(0);
            cube4.setPosition(1);
        }
        if (gamepad2.right_trigger>0.5){
            cube3.setPosition(.5);
            cube4.setPosition(.5);
        }*/
        if (gamepad2.a){
            liftLeft.setPosition(0);
            liftRight.setPosition(0);
        }
        if (gamepad2.b){
            liftLeft.setPosition(.3/3 + .02);
            liftRight.setPosition(.3 * 18.8/20/3 + .02); // sync motors
        }
        if (gamepad2.y){
            liftLeft.setPosition(.6/3 + .02);
            liftRight.setPosition(.6 * 18.8/20/3 + .02); // sync motors
        }
        if (gamepad2.x){
            liftLeft.setPosition(.3 + .02);
            liftRight.setPosition(.3 * 18.8/20 + .02); // sync motors
        }

       // Telemetry();
    }
}
