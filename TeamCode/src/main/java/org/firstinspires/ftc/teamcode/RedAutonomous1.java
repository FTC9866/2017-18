package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="RedAutonomous1", group="TeleOp")

public class RedAutonomous1 extends VirusMethods {
    enum state  {dropArm,scanJewel,knockJewelRight, knockJewelLeft, stop}
    state state;
    public void start(){
        state=state.dropArm;
        lmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    @Override
    public void loop(){
        switch (state) {
            case dropArm:
                jewelKnocker.setPosition(1);
                waitTime(1000);
                resetEncoder();
                state=state.scanJewel;
            break;

            case scanJewel:
                if (colorSensor.red() < colorSensor.blue()) {
                    state=state.knockJewelRight;

                }
                else if (colorSensor.blue() < colorSensor.red()) {
                    state=state.knockJewelLeft;
                }
            break;

            case knockJewelRight:
                if (setMotorPositionsINCH(-3,-3,-3,-3,-1)){
                    state=state.stop;
                }
                break;

            case knockJewelLeft:
                if (setMotorPositionsINCH(3,3,3,3,1)){
                    state=state.stop;
                }
                break;

            case stop:
                telemetry.addData("done","done");
                runMotors(0,0,0,0);
                break;
        }
        Telemetry();
    }
}