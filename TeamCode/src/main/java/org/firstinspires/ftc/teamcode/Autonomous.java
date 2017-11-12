package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Autonomous", group="TeleOp")

public class Autonomous extends VirusMethods {
    enum state  {dropArm,knockJewel,stop}
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
                state=state.knockJewel;
            break;

            case knockJewel:
                if (colorSensor.red() < colorSensor.blue()) {
                    setMotorPositionsINCH(1,1,1,1,1);
                    state=state.stop;

                }
                else if (colorSensor.blue() < colorSensor.red()) {
                    setMotorPositionsINCH(-1,-1,-1,-1,-1);
                    state=state.stop;
                }
            break;

            case stop:
                telemetry.addData("done","done");
                runMotors(0,0,0,0);
                break;
        }
    }
}