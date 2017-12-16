package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mzhang on 11/17/2017.
 */
@TeleOp(name="test", group="TeleOp")

public class test extends VirusMethods{
    enum state  {goToPosition,scanJewel,knockJewelRight, knockJewelLeft, stop, debug}
    state state;

    public void start(){
        lmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetEncoder();
        state=state.goToPosition;

    }
    @Override
    public void loop(){
      switch (state) {
          case goToPosition:
              if (turn(90,1)) {
                  state=state.stop;
              }
              break;
          case stop:
              runMotors(0, 0, 0, 0);
              telemetry.addData("state", state);
              break;

        }
        telemetry.addData("angle remaining", angleRel);;

    }


}
