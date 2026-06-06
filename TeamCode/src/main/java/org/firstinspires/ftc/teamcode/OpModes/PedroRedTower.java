
package org.firstinspires.ftc.teamcode.OpModes; // make sure this aligns with class location

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.MSParams;
import org.firstinspires.ftc.teamcode.Libs.MSMechOps;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RedTowerMEMEMEMEMEMMEMEMEMEMMEMEMEME", group = "Examples")
public class PedroRedTower extends LinearOpMode {

    private HWProfile2 robot = new HWProfile2();
    public final static MSParams params = new MSParams();
    private LinearOpMode myOpMode = this;
//    public HardwareMap hwmap;
    private MSMechOps mechOps;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean GateClear;
    private boolean NoGateClear;
    private int pathState;

    private final Pose startPose = new Pose(117.8, 130, Math.toRadians(36)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(84, 84, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose PrescorePose = new Pose(90, 90, Math.toRadians(40)); // Scoring Pose22 of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1PoseEnd = new Pose(121, 82, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1PoseBegin= new Pose(96, 86, Math.toRadians(0));
    private final Pose pickup2PoseBegin = new Pose(96, 61, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2PoseEnd = new Pose(124, 57, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3PoseBegin = new Pose(96, 37, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup3PoseEnd = new Pose(124, 32, Math.toRadians(0)); // 180 PedroRedTowerLowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose moveGatePoseClear = new Pose(119, 73, Math.toRadians(110));
    private final Pose GatePoseClear = new Pose(126.5, 71, Math.toRadians(0));
    private final Pose endPose = new Pose(94, 53, Math.toRadians(0)); // 135 End Position of the Robot

    //private Path scorePreload;
    private PathChain scorePreload,scoreScore, scorePickup1, grabPickup1Begin,grabPickup1End, moveToGateClear, gateClear, grabPickup2Begin,grabPickup2End,reversePose2, scorePickup2, grabPickup3Begin, grabPickup3End, scorePickup3,endingPose;

    public void runOpMode() {

        telemetry.addLine("ready to get started");
        telemetry.update();
        sleep(500);


        robot.init(hardwareMap, false);

        telemetry.addLine("Hardware is initialized!!!");
        telemetry.update();
        robot.servoFLIPPER.setPosition(params.flipper_stop);
        sleep(500);

        mechOps = new MSMechOps(robot, myOpMode, params);

        telemetry.addLine("MechOps is initialized!!!");
        telemetry.update();
        sleep(500);

        robot.pinpoint.resetPosAndIMU();
        telemetry.addLine("IMU is Calibrated!!!");
        telemetry.update();
        sleep(500);

        follower = Constants.createFollower(hardwareMap);
        telemetry.addLine("Follower is initialized!!!");
        telemetry.update();
        sleep(500);

        buildPaths();
        telemetry.addLine("BuildPaths is initialized!!!");
        telemetry.update();
        sleep(500);

        follower.setStartingPose(startPose);
        follower.update();



        // These loop the movements of the robot, these must be called continuously in order to work
        //follower.update();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        GateClear = true;
        NoGateClear = false;

        while (!isStarted() && !isStopRequested()) {
            // Check for gamepad A button press
            if (gamepad1.aWasPressed()) {
                GateClear = !GateClear;
            }
            // Check for gamepad B button press
            if (gamepad1.bWasPressed()) {
                NoGateClear = !GateClear;
            }

            // Update telemetry
            telemetry.addData("Status", "Initialized - Use A/B to select Third Line");
            telemetry.addData("AUTO?", GateClear ? "Gate" : "No Gate");


            telemetry.addLine("Initialization is complete");
            telemetry.addLine("Press Start to Play");
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive() || pathState != -1) {

            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();


        }

        requestOpModeStop();

    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload =follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        moveToGateClear =follower.pathBuilder()
                .addPath(new BezierCurve(pickup1PoseEnd,new Pose(114,79),moveGatePoseClear))
                .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), moveGatePoseClear.getHeading())
                .build();

/* Here is an example for Constant Interpolation
scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1Begin = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1PoseBegin))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1PoseBegin.getHeading())
                .build();

        grabPickup1End = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseBegin, pickup1PoseEnd))
                .setLinearHeadingInterpolation(pickup1PoseBegin.getHeading(), pickup1PoseEnd.getHeading())

                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1PoseEnd,new Pose(94,66),scorePose))
                .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(),scorePose.getHeading())
                .build();

        scoreScore = follower.pathBuilder()
                .addPath(new BezierLine(PrescorePose, scorePose))
                .setLinearHeadingInterpolation(PrescorePose.getHeading(),scorePose.getHeading())
                .build();


        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2Begin = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2PoseBegin))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2PoseBegin.getHeading())
                .build();
        grabPickup2End = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseBegin, pickup2PoseEnd))
                .setLinearHeadingInterpolation(pickup2PoseBegin.getHeading(), pickup2PoseEnd.getHeading())
                .build();
        reversePose2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseEnd, pickup2PoseBegin))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), pickup2PoseBegin.getHeading())
                .build();
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseEnd,new Pose(90,58),scorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(),scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3Begin = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3PoseBegin))
                //.setHeadingConstraint(pickup3PoseBegin.getHeading())
                .setConstantHeadingInterpolation(0)
                .build();
        grabPickup3End = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseBegin, pickup3PoseEnd))
                .setLinearHeadingInterpolation(pickup3PoseBegin.getHeading(), pickup3PoseEnd.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3PoseEnd,new Pose(83,51),scorePose))
                .setLinearHeadingInterpolation(pickup3PoseEnd.getHeading(),scorePose.getHeading())
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        endingPose = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                telemetry.addLine("calling ShooterControl");
                telemetry.update();
                mechOps.intake(1);
                mechOps.shooterControl(params.ShootAutoTower);

                follower.followPath(scorePreload, .85, true);
                follower.update();

                setPathState(1);
                break;
            case 1:

        /* You could check for
        - Follower State: "if(!follower.isBusy()) {}"
        - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
        - Robot Position: "if(follower.getPose().getX() > 36) {}"
        */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    safeWaitSeconds(.01);
                    mechOps.feedShooter(params.Feeder_ON);
                    robot.servoFLIPPER.setPosition(params.flipper_clear);
                    mechOps.intake(1);
                    safeWaitSeconds(params.AutoShooterTime);
                    mechOps.feedShooter(0);
                    robot.servoFLIPPER.setPosition(params.flipper_stop);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1Begin,true);
                    setPathState(2);
                }
                break;

            case 2:
                    if (!follower.isBusy()) {
                        //turning intake on
                        mechOps.intake(1);
                        mechOps.feedShooter(.5);

                        follower.followPath(grabPickup1End,true);
                        setPathState(3);
                    }
                    break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    if (GateClear) {
                        follower.followPath(moveToGateClear, true);
                        setPathState(4);
                    } else {
                        follower.followPath(scorePickup1, true);
                        NoGateClear = true;
                        setPathState(5);
                    }
                    robot.servoFLIPPER.setPosition(params.flipper_stop);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    mechOps.intake(1);

                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, 1,true);
                    //follower.followPath(scoreScore, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    safeWaitSeconds(.01);
                    mechOps.feedShooter(params.Feeder_ON);
                    robot.servoFLIPPER.setPosition(params.flipper_clear);
                    safeWaitSeconds(params.AutoShooterTime);
                    mechOps.feedShooter(0);
                    robot.servoFLIPPER.setPosition(params.flipper_stop);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    mechOps.intake(1);
                    mechOps.feedShooter(.5);
                    follower.followPath(grabPickup2Begin, true);
                    follower.followPath(grabPickup2End, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    //follower.followPath(reversePose2, true);
                    follower.followPath(scorePickup2,1,true);
                    //follower.followPath(scoreScore, true);
                    setPathState(8);
                }
                break;
            case 7:
                if (!follower.isBusy()) {

                    follower.followPath(scoreScore, true);
                    setPathState(8);
                }
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    safeWaitSeconds(.01);
                    mechOps.feedShooter(params.Feeder_ON);
                    robot.servoFLIPPER.setPosition(params.flipper_clear);
                    mechOps.intake(1);
                    safeWaitSeconds(params.AutoShooterTime);
                    mechOps.feedShooter(0);
                    robot.servoFLIPPER.setPosition(params.flipper_stop);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    mechOps.intake(1);
                    mechOps.feedShooter(.5);
                    follower.followPath(grabPickup3Begin, true);
                    follower.followPath(grabPickup3End, true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, 1,true);
                    //follower.followPath(scoreScore, true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    safeWaitSeconds(.01);
                    mechOps.feedShooter(params.Feeder_ON);
                    robot.servoFLIPPER.setPosition(params.flipper_clear);
                    mechOps.intake(1);
                    safeWaitSeconds(params.AutoShooterTime);
                    mechOps.feedShooter(0);
                    robot.servoFLIPPER.setPosition(params.flipper_stop);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(endingPose, true);
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                   mechOps.intake(0);
                   mechOps.feedShooter(0);
                   mechOps.shooterControl(0);
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/


    //method to wait safely with stop button working if needed. Use this instead of sleep
public void safeWaitSeconds(double time) {
    ElapsedTime timer = new ElapsedTime(SECONDS);
    timer.reset();
    while (timer.time() < time) {
    }
}

}
