package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;


@TeleOp(name="BlueAutonomous1", group="TeleOp")

public class BlueAutonomous1 extends VirusMethods {
    enum state  {dropArm,scanJewel,knockJewelRight, knockJewelLeft, stop, goToPosition, debug, alignStraight, toCryptoBox, backOnStone, faceCryptoBox, placeGlyph, turnBackLeft, turnBackRight, turnBack, toCryptoBoxpart1, turn90, toCryptoBoxpart2, moveUntilScanned}
    state state;
    boolean setMotor;
    boolean knock;

    public void init() {
        super.init();
        liftLeft.setPosition(0);
        liftRight.setPosition(0);
        jewelKnocker.setPosition(0);
        cube1.setPosition(.5);
        cube2.setPosition(.5);
    }

    public void start() {
        lmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
                    knock  = true;
                    colorSensor.enableLed(false);
                    state=state.knockJewelLeft;
                }
                else if (colorSensor.blue() < colorSensor.red()) {
                    knock = false;
                    state=state.knockJewelRight;
                }
                break;

            case knockJewelLeft:
                if (turnMotorsPlus(345, 0.3)){
                    jewelKnocker.setPosition(0);
                    state=state.turnBack;
                }
                break;

            case knockJewelRight:
                if (turnMotorsPlus(15,0.3)) {
                    jewelKnocker.setPosition(0);
                    state = state.turnBack;
                }
                break;
            case turnBack:
                if (turnMotorsPlus(0, 0.3)){
                    position = lmotor0.getCurrentPosition();
                    state = state.moveUntilScanned;
                }
                break;
            //turnBackLeft and turnBackRight kept just in case turnMotorsPlus method doesn't work
            case turnBackLeft:
                turnMotors(0, true, 0.3);
                state = state.moveUntilScanned;
                break;

            case turnBackRight:
                turnMotors(0, false, 0.3);
                state = state.moveUntilScanned;
                break;

            case moveUntilScanned:
                runMotors(.1,.1,.1,.1); //program to make it move backwards if it doesn't see it after traveling a certain distance
                    if(vuMark != RelicRecoveryVuMark.UNKNOWN){
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        VuMarkStored = vuMark;
                        amountMovedForward = (lmotor0.getCurrentPosition()-position)*inPerPulse; //how many inches it moved back to scan the vision target
                        state = state.alignStraight;
                    }
                break;
            case alignStraight:
                if (turnMotorsPlus(0,1)) {
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
                lift(0.06); //so that cube doesn't drag on ground
                if (VuMarkStored == RelicRecoveryVuMark.LEFT){
                    if (setMotorPositionsINCH(-30-amountMovedForward,-30-amountMovedForward,-30-amountMovedForward,-30-amountMovedForward, -.5)){
                        resetEncoder();
                        telemetry.addData("reee", "e");
                        state=state.faceCryptoBox;
                    }
                }
                if (VuMarkStored == RelicRecoveryVuMark.CENTER){
                    if (setMotorPositionsINCH(-36-amountMovedForward,-36-amountMovedForward,-36-amountMovedForward,-36-amountMovedForward, -.5)){
                        resetEncoder();
                        state=state.faceCryptoBox;
                    }
                }
                if (VuMarkStored == RelicRecoveryVuMark.RIGHT){
                    if (setMotorPositionsINCH(-44-amountMovedForward,-44-amountMovedForward,-44-amountMovedForward,-44-amountMovedForward, .5)){
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
                if (turnMotorsPlus(270,.75)) {
                    resetEncoder();
                    state=state.placeGlyph;
                }
                break;
            case placeGlyph:
                runMotors(0.5,0.5,0.5,0.5);
                waitTime(500);
                runMotors(0,0,0,0);
                cube1.setPosition(0);
                cube2.setPosition(1);
                runMotors(-0.5,-0.5,-0.5,-0.5);
                waitTime(500);
                runMotors(0,0,0,0);
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
        telemetry.addData("Blue: true Red: false ", knock);
        telemetry.addData("state", state);
        // Telemetry();,k
    }
}