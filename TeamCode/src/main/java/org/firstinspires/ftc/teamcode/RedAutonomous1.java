package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="RedAutonomous1", group="TeleOp")

public class RedAutonomous1 extends VirusMethods {
    enum state  {dropArm,scanJewel,knockJewelRight, knockJewelLeft, stop, goToPosition, debug}
    state state;
    boolean setMotor;
    public void start(){
        lmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        state=state.dropArm;
    }
    @Override
    public void loop(){
        switch (state) {
            case dropArm:
                jewelKnocker.setPosition(0.75);
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
                if (setMotorPositionsINCH(12,12,12,12,1)){
                    state=state.stop;
                }
                break;

            case knockJewelLeft:
                if (setMotorPositionsINCH(-12,-12,-12,-12,-1)){
                    state=state.stop;
                }
                break;

            case debug:
                //telemetry.addData("done","done");
                telemetry.addData("setMotor returns", setMotor);
                telemetry.addData("inPerPulse", inPerPulse);
                telemetry.addData("left motor", lmotor0.isBusy());
                telemetry.addData("counter",counter);
                break;

            case stop:
                telemetry.addData("state", state);
                //runMotors(0,0,0,0);
                break;
        }
       // Telemetry();
    }
}