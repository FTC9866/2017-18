package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



/**
 * Created by Andrew on 11/10/2017.
 */
@TeleOp(name="Autonomous", group="TeleOp")

public class Autonomous extends VirusAuto {

    @Override
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