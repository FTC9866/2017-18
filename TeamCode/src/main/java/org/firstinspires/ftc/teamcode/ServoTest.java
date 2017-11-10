package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Keertik on 10/13/2017.
 */
@TeleOp(name="Servo", group="TeleOp")
public class ServoTest extends OpMode {
    Servo servo1;
    Servo servo2;
    public void init(){
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
    }
    public void loop(){
        if (gamepad2.y) {
            servo1.setPosition(1);
        }
        if (gamepad2.x) {
            servo2.setPosition(1);
        }
    }
}
