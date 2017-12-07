package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@TeleOp(name="BlueAutonomous1", group="TeleOp")

public class BlueAutonomous1 extends VirusMethods {
    enum state  {dropArm,scanJewel,knockJewelRight, knockJewelLeft, stop, goToPosition, debug, alignStraight, toCryptoBox, backOnStone, faceCryptoBox, placeGlyph, moveUnitlScanned}
    state state;
    boolean setMotor;

    public void start() {
        lmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cube1.setPosition(.5);
        cube2.setPosition(.5);
        state=state.dropArm;
        vuforiaInit();
    }
    @Override
    public void loop() {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            double tX = trans.get(0);
            double tY = trans.get(1);
            double tZ = trans.get(2);

            // Extract the rotational components of the target relative to the robot
            double rX = rot.firstAngle;
            double rY = rot.secondAngle;
            double rZ = rot.thirdAngle;
        }
        switch (state) {
            case dropArm:

                jewelKnocker.setPosition(0.65);
                colorSensor.enableLed(true);
                waitTime(1000);
                resetEncoder();
                state=state.scanJewel;
                break;

            case scanJewel:
                if (colorSensor.red() < colorSensor.blue()) { //checks to see if object is more red or more blue
                    colorSensor.enableLed(false);
                    state=state.knockJewelRight;
                }
                else if (colorSensor.blue() < colorSensor.red()) {
                    state=state.knockJewelLeft;
                }
                break;

            case knockJewelRight:
                if (setMotorPositionsINCH(3,3,3,3,.5)){
                    jewelKnocker.setPosition(0);
                    state=state.moveUnitlScanned;
                }
                break;

            case knockJewelLeft:
                if (setMotorPositionsINCH(-3,-3,-3,-3,-.5)){
                    jewelKnocker.setPosition(0);
                    state=state.moveUnitlScanned;
                }
                break;
            case moveUnitlScanned:
                runMotors(.1,.1,.1,.1); //program to make it move backwards if it doesn't see it after traveling a certain distance
                    if(vuMark != RelicRecoveryVuMark.UNKNOWN){
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        VuMarkStored = vuMark;
                        state = state.alignStraight;
                    }
                break;
            case alignStraight:
                if (turn(0,1)) {
                    resetEncoder();
                    counter = 0;
                    state = state.toCryptoBox ;
                }
                break;
            case backOnStone: // broken plz fix
                if (setMotorPositions(0,0,0,0, .5)){
                    resetEncoder();
                    state=state.toCryptoBox;
                }
                break;
            case toCryptoBox:
                liftLeft.setPosition(0.01); //so that cube doesn't drag on ground
                liftRight.setPosition(0.01);
                if (VuMarkStored == RelicRecoveryVuMark.LEFT){
                    if (setMotorPositionsINCH(-30,-30,-30,-30, -.5)){
                        resetEncoder();
                        telemetry.addData("reee", "e");
                        state=state.faceCryptoBox;
                    }
                }
                if (VuMarkStored == RelicRecoveryVuMark.CENTER){
                    if (setMotorPositionsINCH(-36,-36,-36,-36, -.5)){
                        resetEncoder();
                        state=state.faceCryptoBox;
                    }
                }
                if (VuMarkStored == RelicRecoveryVuMark.RIGHT){
                    if (setMotorPositionsINCH(-44,-44,-44,-44, .5)){
                        resetEncoder();
                        state=state.faceCryptoBox;
                    }
                }
                else {
                    //nani
                    //program to park back onto balance stone (li)?
                }
                break;
            case faceCryptoBox:
                if (turn(270,.75)) {
                    resetEncoder();
                    state=state.placeGlyph;
                }
                break;
            case placeGlyph:
                runMotors(1,1,1,1);
                waitTime(500);
                runMotors(0,0,0,0);
                cube1.setPosition(0);
                cube2.setPosition(1);
                state = state.stop;
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
                runMotors(0,0,0,0);
                break;
        }
        telemetry.addData("state", state);
        // Telemetry();,k
    }
}