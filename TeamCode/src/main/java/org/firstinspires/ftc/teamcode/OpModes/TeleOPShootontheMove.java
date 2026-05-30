/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.MSParams;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;
import org.firstinspires.ftc.teamcode.Libs.MSMechOps;

import java.util.Locale;
import java.util.Vector;
/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: TeleOp Shoot on the Move", group="Competition")
public class TeleOPShootontheMove extends LinearOpMode {

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final static HWProfile2 robot = new HWProfile2();
    private final LinearOpMode opMode = this;
    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
    public final static MSParams params = new MSParams();
    public MSMechOps mechOps = new MSMechOps(robot, opMode, params);

    private double testPosition = 0;
    private double shooterVel = 0;
    private double triggerRPM = 0;
    private double artDistance = 0;
    private double aftDistance = 0;
    public boolean AllianceBlue = true;
    public boolean FieldC = false;
    public double HA = 0.6;
    public double headingGoal = 45; // Radians
    public double headingError; // Radians
    PIDFController controller = new PIDFController((new PIDFCoefficients(0.78, 0, 0.03, 0.07)));
    public double smallDiff;
    public double rpmLED;
    public double VelAdj;
    public boolean isRed;
    public boolean AutoVel = true;



    public void runOpMode() {
        robot.init(hardwareMap, true);
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
        robot.servoFLIPPER.setPosition(params.flipper_stop);
        robot.servoLIFT.setPosition(params.LIFTZero);
        ElapsedTime Climb_Timer = new ElapsedTime();
        // robot.pinpoint.recalibrateIMU();  Removed, Pinpoint should keep posiiton after Auto

        robot.LredLED.setMode(DigitalChannel.Mode.OUTPUT);
        robot.LgreenLED.setMode(DigitalChannel.Mode.OUTPUT);
        robot.RredLED.setMode(DigitalChannel.Mode.OUTPUT);
        robot.RgreenLED.setMode(DigitalChannel.Mode.OUTPUT);
// Added to check if pinpoint has position data, if not should st in center of field
        Pose2D startpos = new Pose2D(DistanceUnit.INCH,72,72,AngleUnit.DEGREES,0);

        robot.pinpoint.update();
        Pose2D check = robot.pinpoint.getPosition();
        String initdata = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", check.getX(DistanceUnit.INCH), check.getY(DistanceUnit.INCH), check.getHeading(AngleUnit.DEGREES));



        telemetry.update();
        // Loop while OpMode is not started and not stopped
        while (!isStarted() && !isStopRequested()) {
            // Check for gamepad A button press
            if (gamepad1.aWasPressed()) {
                isRed = !isRed;
            }
            // Check for gamepad B button press
            if (gamepad1.bWasPressed()) {
                isRed = !isRed;
            }
            if (gamepad1.dpadUpWasPressed()){

                //reset IMU and wait .5 seconds

                robot.pinpoint.setPosition(startpos);
            }

            // Update telemetry
            telemetry.addData("Position", initdata);
            telemetry.addLine(" Press D Pad UP to set to 72,72,0");
            telemetry.addData("Status", "Initialized - Use A/B to select color");
            telemetry.addData("Alliance", isRed ? "Red" : "Blue");
            telemetry.update(); // Push the telemetry data to the Driver Station

            // Add a small pause to prevent the loop from running excessively fast
            sleep(100);
        }
// Wait for the game to start (driver presses PLAY) Not needed when While (!is Started is used
        //       waitForStart();


        // run until the end of the match (driver presses STOP)
        double leftPower = 0;
        double rightPower = 0;
        ElapsedTime buttonPressTimer = new ElapsedTime();
        boolean flipperDown = false;
        boolean HiSpeed = false;
        int climbGrabStage = 1;
        double flipperPostition = params.flipper_stop;
        double intakePower = params.Intake_OFF;

//        double spicePosition = params.SPICE_CLOSE;
//        double TwistPosition = params.TWIST_HORIZONTAL;
        double botHeading;
        double x, y, rx;
        double rotX, rotY;
        double denominator, frontLeftPower, backLeftPower, frontRightPower, backRightPower;
        double powerFactor = 1;
        double shooterPower = 0;
        double artDist;
        double aftDist;
        boolean headingLock = false;
        VelAdj = 0;


        while (opModeIsActive()) {

            /* ###########################################
               #                                         #
               # New field centric drive using PinPoint  #
               #                                         #
               ###########################################*/


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
//            if (gamepad1.options) {
//                robot.pinpoint.recalibrateIMU();
//                //recalibrates the IMU without resetting position
//            }
            if (gamepad1.x) {
                //            shooterPower = 1;
                shooterVel = params.ShootTeleFar;
                rpmLED = .666;
                HA=params.Hood1Far;
            }
            if (gamepad1.b) {
                shooterVel = params.ShootTeleNear;
                rpmLED = .333;
                HA=params.Hood1Close;
            }
            if (gamepad1.a) {
                //           shooterPower = 0;
                shooterVel = 0;
                VelAdj =0;
                rpmLED = 0;

            }
            if (gamepad1.right_bumper) {
                AutoVel = !AutoVel;

            }

            if (gamepad1.yWasPressed()) {
                isRed = !isRed;
            }

            if (gamepad1.left_stick_button) {
                robot.servoLIFT.setPosition(params.LIFTlifting);
            }

            if (gamepad1.right_stick_button) {
                robot.servoLIFT.setPosition(params.LIFTZero);
            }

            if (gamepad1.right_trigger > .25) {
                robot.servoFLIPPER.setPosition(params.flipper_clear);
                mechOps.feedShooter(params.Feeder_ON);
                mechOps.intake(params.Intake_ON);
            } else if (gamepad1.left_trigger > .25) {
                robot.servoFLIPPER.setPosition(params.flipper_rev);
                mechOps.feedShooter(params.Feeder_REV);
                mechOps.intake(params.Intake_Rev);
            } else {
                robot.servoFLIPPER.setPosition(params.flipper_stop);
                mechOps.feedShooter(params.Feeder_OFF);
            }

            if (gamepad1.dpad_right) {
                mechOps.intake(params.Intake_ON);
//                if((buttonPressTimer.time() > 0.25) && intakeOff){
//                    intakePower = params.Intake_OFF;
//                    intakeOff= false;
//                    buttonPressTimer.reset();
//                } else if(buttonPressTimer.time() > 0.25) {
//                    intakePower = params.Intake_ON;
//                    intakeOff = true;
//                    buttonPressTimer.reset();
//                }
            }

            if (gamepad1.dpad_left) {
                mechOps.intake(params.Intake_OFF);
            }

            if (gamepad1.psWasPressed()){
                if(isRed){
                    robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,8,47,AngleUnit.DEGREES,0));
                }
                else {
                    robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,136,47,AngleUnit.DEGREES,180));
                }
            }

            if (gamepad1.dpad_down) {
                if ((buttonPressTimer.time() > 0.25)) {
                    //               shooterPower = shooterPower - 0.05;
                    VelAdj= VelAdj - 20;
                    rpmLED = rpmLED - .02;
                    buttonPressTimer.reset();
                }

            }
            if (gamepad1.dpad_up) {
                if ((buttonPressTimer.time() > 0.25)) {
                    //               shooterPower = shooterPower + 0.05;
                    VelAdj=VelAdj + 20;
                    rpmLED = rpmLED + .02;
                    buttonPressTimer.reset();
                }

            }
            if (gamepad2.dpad_down) {
                if ((buttonPressTimer.time() > 0.25)) {
                    //               shooterPower = shooterPower - 0.05;
                    HA=HA-.02;

                    buttonPressTimer.reset();
                }

            }
            if (gamepad2.dpad_up) {
                if ((buttonPressTimer.time() > 0.25)) {
                    //               shooterPower = shooterPower + 0.05;
                    HA=HA+.02;

                    buttonPressTimer.reset();
                }

            }
/**
 if (gamepad1.right_bumper) {
 if((buttonPressTimer.time() > 0.25) && flipperDown){
 flipperPostition = params.flipper_down;
 flipperDown= false;
 buttonPressTimer.reset();
 } else if(buttonPressTimer.time() > 0.25) {
 flipperPostition = params.flipper_up;
 flipperDown = true;
 buttonPressTimer.reset();
 }

 }
 **/
            if (gamepad1.leftBumperWasPressed()) {
                headingLock = !headingLock;
                //controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
                //controller.updateError(getHeadingError());
            }
            if (gamepad1.optionsWasPressed()) {
                FieldC = !FieldC;
                //controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
                //controller.updateError(getHeadingError());
            }


            robot.pinpoint.update();
            Pose2D pos = robot.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            double xvel = robot.pinpoint.getVelX(DistanceUnit.INCH);
            double yvel = robot.pinpoint.getVelY(DistanceUnit.INCH);

            if (AutoVel) {
                //  Overrides Shooter Velocity with auto calculated velocity
                shooterVel = gettargetVel(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH),xvel,yvel);
                shooterVel=shooterVel+VelAdj;
                if(pos.getY(DistanceUnit.INCH)>48){
                    rpmLED = .444;
                }
                else {
                    rpmLED = .611;
                }

            }

            if (FieldC) {

                // botHeading = robot.pinpoint.getHeading(AngleUnit.RADIANS);
                if (isRed) {
                    botHeading = pos.getHeading(AngleUnit.RADIANS);
                }else{
                    botHeading =  pos.getHeading(AngleUnit.RADIANS)-Math.PI;
                }



            } else {
                botHeading = 0;

            }

            y = -gamepad1.left_stick_y;    //removed   + gamepad1.right_stick_y so no more drift?
            x = gamepad1.left_stick_x;

            headingGoal = getnewHeadingGoal(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH),xvel,yvel);

            if (headingLock) {
                //smallDiff = getSmallestSignedAngleDifference(robot.pinpoint.getHeading(AngleUnit.DEGREES), headingGoal);
                double error = headingGoal - pos.getHeading(AngleUnit.DEGREES);

                if (error > 180) {
                    error -= 360;
                } else if (error < -180) {
                    error += 360;

                }

                rx = 0.02 * -error;
                rx = Math.min(Math.max(rx, -0.4), 0.4);

                robot.RgreenLED.setState(true);
                robot.RredLED.setState(false);
                robot.LgreenLED.setState(true);
                robot.LredLED.setState(false);


            } else {

                rx = gamepad1.right_stick_x;

                robot.RgreenLED.setState(false);
                robot.RredLED.setState(true);
                robot.LgreenLED.setState(false);
                robot.LredLED.setState(true);

            }


            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftPower = (rotY + rotX + rx) / denominator;
            backLeftPower = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower = (rotY + rotX - rx) / denominator;

            robot.motorLF.setPower(frontLeftPower * powerFactor);
            robot.motorLR.setPower(backLeftPower * powerFactor);
            robot.motorRF.setPower(frontRightPower * powerFactor);
            robot.motorRR.setPower(backRightPower * powerFactor);
//            robot.motorShooter.setPower(shooterPower);
//            robot.motorShooter.setVelocity(angularRate);
            shooterControl(shooterVel+VelAdj);
            //mechOps.hoodAngle(HA);
            robot.servoRPMLight.setPosition(rpmLED);
            //artDist = robot.ArtSensor.getDistance(DistanceUnit.CM);
            //aftDist = robot.AftSensor.getDistance(DistanceUnit.CM);

//            if(artDist>13 && aftDist>13) {
//                robot.RgreenLED.setState(true);
//                robot.RredLED.setState(true);
//                robot.LgreenLED.setState(true);
//                robot.LredLED.setState(true);
//            }else if ((artDist<13 && aftDist<13)){
//                    robot.RgreenLED.setState(true);
//                    robot.RredLED.setState(false);
//                    robot.LgreenLED.setState(true);
//                    robot.LredLED.setState(false);
//                }else {
//                    robot.RgreenLED.setState(false);
//                    robot.RredLED.setState(true);
//                    robot.LgreenLED.setState(false);
//                    robot.LredLED.setState(true);
//                }


            telemetry.addData("Alliance", isRed ? "Red" : "Blue");
            telemetry.addData("Position", data);
            telemetry.addData("Heading Goal", headingGoal);
            //telemetry.addData("shooterPower = ",shooterPower);
            //telemetry.addData("Left Front Motor Encoder = ", robot.motorLF.getCurrentPosition());
            //telemetry.addData("Left Front Motor Current = ", robot.motorLF.getCurrent(CurrentUnit.AMPS));
            //telemetry.addData("Left Rear Motor Encoder = ", robot.motorLR.getCurrentPosition());
            //telemetry.addData("Left Rear Motor Current = ", robot.motorLR.getCurrent(CurrentUnit.AMPS));
            //telemetry.addData("Right Front Motor Encoder = ", robot.motorRF.getCurrentPosition());
            //telemetry.addData("Right Front Motor Current = ", robot.motorRF.getCurrent(CurrentUnit.AMPS));
            //telemetry.addData("Right Rear Motor Encoder = ", robot.motorRR.getCurrentPosition());
            //telemetry.addData("Right Rear Motor Current = ", robot.motorRR.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("---------------------------------");
            //telemetry.addData("Shooter Amps = ", robot.motorShooter.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Shooter Vel Act= ", robot.motorShooter.getVelocity());
            telemetry.addData("Shooter Vel Set = ", shooterVel);
            telemetry.addData(" Vel Adj = ", VelAdj);
            telemetry.addLine("---------------------------------");
            //telemetry.addData("Feeder Vel Act= ", robot.motorFeeder.getVelocity());
            //telemetry.addData("Feeder Vel Set = ", params.Feeder_ON);
            telemetry.addData("TestPosition = ", testPosition);
            //telemetry.addData("HOOD ANGLE", HA);
            //telemetry.addData("ArtSensor",robot.ArtSensor.getDistance(DistanceUnit.CM));
            //telemetry.addData("AftSensor",robot.AftSensor.getDistance(DistanceUnit.CM));
            telemetry.addLine("---------------------------------");
            //telemetry.addData("Y stick Output", rx);
            telemetry.addData("HeadingLock?", headingLock);
            telemetry.addData("Field Centric?", FieldC);
            //telemetry.addData("Small Diff",smallDiff);
            telemetry.addLine("---------------------------------");

            telemetry.addData("Status", "Running");
            //telemetry.addData("Left Power", leftPower);
            //telemetry.addData("Right Power", rightPower);

            telemetry.addData("Eli Pink Shirt", "yes");
            telemetry.update();
            panelsTelemetry.addData("Robot Shooter Velocity =", shooterVel);
            panelsTelemetry.update();
        }
    }

    /**
     * Method shooterControl()
     *
     * @param
     */
    public void shooterControl(double targetVel) {
        robot.motorShooter.setVelocity((targetVel));
        robot.motorShooterTop.setVelocity((targetVel));
    }   // end of method shooterControl

    public double getHeadingError() {
        //       if (follower.currentPath == null) {
        //           return 0;
        //      }

        headingError = MathFunctions.getTurnDirection(follower.getPose().getHeading(), headingGoal) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), headingGoal);
        return headingError;
    }

    public double getSmallestSignedAngleDifference(double currentAngle, double targetAngle) {
        double angleDifference = targetAngle - currentAngle;
        // Use Math.atan2 to normalize the angle difference to the range [-PI, PI]
        double smallestDifference = Math.atan2(Math.sin(angleDifference), Math.cos(angleDifference));
        return smallestDifference;
    }

    /**
     * method rpmToTicksPerSecond
     *
     * @param targetRPM
     */
    private double rpmToTicksPerSecond(double targetRPM) {
        return (targetRPM * 28 / 60);
    }   // end of method rpmToTicksPerSecond

    public double getnewHeadingGoal(double currentx, double currenty,double velx,double vely) {

        double futurex=velx* params.BallAirTime;
        double x = -(currentx + futurex);
            if (isRed) {
                x = 144 - (currentx + futurex);
            }

        double futurey=vely* params.BallAirTime;
            double y = 144 - (currenty+futurey);


        double newHeadingGoal = Math.toDegrees(Math.atan2(y, x));

        return newHeadingGoal;
    }

    public double gettargetVel(double currentx, double currenty,double velx,double vely) {
        double futurex=velx* params.BallAirTime;
        double x = -(currentx + futurex);


        if (isRed) {
            x = 144 - (currentx + futurex);

        }

        double futurey=vely* params.BallAirTime;
        double y = 144 - (currenty+futurey);
        double dist = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double targetVel =1;

        if(currenty>48){
            targetVel = ((dist* 1.6351) + 1417.5) - 100; //near slope added offset of 100 until new curve
        }
        else {
            targetVel =  ((dist * 1.647) + 1481) - 80;//far slope added offset of 80 until new curve
        }
        return targetVel;
    }
}
