
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

@Autonomous(name = "WallRedStraight", group = "Examples")
public class REDwallStraight extends LinearOpMode {

    private HWProfile2 robot = new HWProfile2();
    public final static MSParams params = new MSParams();
    private LinearOpMode myOpMode = this;
//    public HardwareMap hwmap;
    private MSMechOps mechOps;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private boolean thirdline;
    private boolean round2;
    private int pathState;


    private final Pose startPose = new Pose(88, 9.5, Math.toRadians(68)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(88, 18, Math.toRadians(68)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose PrescorePose = new Pose(92, 18, Math.toRadians(68)); // Scoring Pose22 of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup3PoseEnd = new Pose(122, 80, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3PoseBegin= new Pose(100, 80, Math.toRadians(0));
    private final Pose pickup2PoseBegin = new Pose(97.5, 10, Math.toRadians(0));// Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2PoseSlide = new Pose(134.5, 7, Math.toRadians(-10));//y9.5 moved to prevent slide
    private final Pose pickup2PoseEnd = new Pose(134.5, 12, Math.toRadians(45)); // x-135.5 to prevent getting stuck it was moved by .5
    private final Pose pickup2PoseScore = new Pose(88, 17, Math.toRadians(74)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose AA1Pose = new Pose(130, 17, Math.toRadians(-20)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose AA1PoseScoop = new Pose(132, 8, Math.toRadians(0));// 180 PedroRedTowerLowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose AA1Poseround2 = new Pose(132, 9, Math.toRadians(10));// 180 PedroRedTowerLowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup1PoseBegin = new Pose(100, 33, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup1PoseEnd = new Pose(133, 33, Math.toRadians(0)); // 180 PedroRedTowerLowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose endPose = new Pose(85, 36, Math.toRadians(90)); // 135 End Position of the Robot

    //private Path scorePreload;
    private PathChain scorePreload,scoreScore, scorePickup1, grabPickup1Begin,grabPickup1End, grabPickup2Begin,grabPickup2Slide,grabPickup2End,grabPickup2Shoot, scorePickup2, grabPickup3Begin, grabPickup3End, scorePickup3,endingPose,AA1,AA1Scoop,AA1round2,AA1ToShoot;

    public void runOpMode() {

        telemetry.addLine("ready to get started");
        telemetry.update();
        sleep(500);


        robot.init(hardwareMap, false);

        telemetry.addLine("Hardware is initialized!!!");
        telemetry.update();
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
        thirdline = true;
        round2 = false;

        while (!isStarted() && !isStopRequested()) {
            // Check for gamepad A button press
            if (gamepad1.aWasPressed()) {
                thirdline = false;
            }
            // Check for gamepad B button press
            if (gamepad1.bWasPressed()) {
                thirdline = true;
            }

            // Update telemetry
            telemetry.addData("Status", "Initialized - Use A/B to select Third Line");
            telemetry.addData("AUTO?", thirdline ? "Third" : "Wall");
            telemetry.addLine("Initialization is complete");
            telemetry.addLine("Press Start to Play");
            telemetry.update();
            sleep(50);
        }

        telemetry.addLine("Initialization is complete");
        telemetry.addLine("Press Start to Play");
        telemetry.update();

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

/* Here is an example for Constant Interpolation
scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        AA1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup2PoseBegin))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup2PoseBegin.getHeading())
                .build();

        AA1Scoop = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseBegin, pickup2PoseSlide))
                .setLinearHeadingInterpolation(pickup2PoseBegin.getHeading(), pickup2PoseSlide.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        AA1round2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseSlide, pickup2PoseEnd))
                .setLinearHeadingInterpolation(pickup2PoseSlide.getHeading(), pickup2PoseEnd.getHeading())
                .build();

        AA1ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseEnd, PrescorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), PrescorePose.getHeading())
                .build();



        scoreScore = follower.pathBuilder()
                .addPath(new BezierLine(PrescorePose, scorePose))
                .setLinearHeadingInterpolation(PrescorePose.getHeading(),scorePose.getHeading())
                .build();
        grabPickup1Begin = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,new Pose(88,37),pickup1PoseBegin))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup1PoseBegin.getHeading())
                .build();

        grabPickup1End = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseBegin, pickup1PoseEnd))
                .setLinearHeadingInterpolation(pickup1PoseBegin.getHeading(), pickup1PoseEnd.getHeading())

                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseEnd, PrescorePose))
                .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(),PrescorePose .getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2Begin = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pickup2PoseBegin))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup2PoseBegin.getHeading())
                .build();
        grabPickup2End = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseSlide, pickup2PoseEnd))
                .setLinearHeadingInterpolation(pickup2PoseSlide.getHeading(), pickup2PoseEnd.getHeading())
                .build();
        grabPickup2Slide = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseBegin, pickup2PoseSlide))
                .setLinearHeadingInterpolation(pickup2PoseBegin.getHeading(), pickup2PoseSlide.getHeading())
                .build();
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2Shoot = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseEnd, PrescorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), PrescorePose.getHeading())
                .build();
        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2PoseEnd, scorePose))
//                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(),scorePose .getHeading())
//
//                .build();
                .addPath(new BezierLine(pickup2PoseScore,PrescorePose))
                .setLinearHeadingInterpolation(pickup2PoseScore.getHeading(),PrescorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3Begin = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,new Pose(72,71),pickup3PoseBegin))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup3PoseBegin.getHeading())
                .build();
        grabPickup3End = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseBegin, pickup3PoseEnd))
                .setLinearHeadingInterpolation(pickup3PoseBegin.getHeading(), pickup3PoseEnd.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseEnd, scorePose))
                .setLinearHeadingInterpolation(pickup3PoseEnd.getHeading(),scorePose .getHeading())

                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        endingPose = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate () {
        switch (pathState) {
            case 0:
                telemetry.addLine("calling ShooterControl");
                telemetry.update();
                //mechOps.intake(1);
                mechOps.shooterControl(params.ShootAutoStart);
                /* Score Preload */
                safeWaitSeconds(1.5);
                mechOps.feedShooter(params.Feeder_ON);
                robot.servoFLIPPER.setPosition(params.flipper_clear);
                mechOps.intake(1);
                safeWaitSeconds(2);
                mechOps.feedShooter(0);
                robot.servoFLIPPER.setPosition(params.flipper_stop);
                mechOps.shooterControl(params.ShootAutoLong);
                follower.followPath(AA1, true);
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
//                    if (!follower.isBusy())
                if (!follower.isBusy() || follower.getCurrentPath().getClosestPointTValue() > 0.85){
                    /* Score Preload */

                    safeWaitSeconds(0.15);
                    //mechOps.feedShooter(params.Feeder_ON);
                    mechOps.intake(1);
                    // safeWaitSeconds(params.AutoShooterTime);
                    mechOps.feedShooter(0.5);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    robot.servoFLIPPER.setPosition(params.flipper_stop);
                    follower.followPath(AA1Scoop, .5, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() || follower.getCurrentPath().getClosestPointTValue() > 0.85){
                    //turning intake on
                    robot.servoFLIPPER.setPosition(params.flipper_stop);

                    mechOps.intake(1);

                    follower.followPath(AA1round2, true);
                    setPathState(3);
                }
                break;

            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy() || follower.getCurrentPath().getClosestPointTValue() > 0.85){
                    //turning intake on
                    safeWaitSeconds(0.25);
                    robot.servoFLIPPER.setPosition(params.flipper_stop);
                    mechOps.intake(1);
                    mechOps.feedShooter(.5);

                    follower.followPath(AA1ToShoot, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    mechOps.feedShooter(0);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy() || follower.getCurrentPath().getClosestPointTValue() > 0.85) {
                    //turning intake on
                    robot.servoFLIPPER.setPosition(params.flipper_clear);
                    mechOps.feedShooter(params.Feeder_ON);
                    mechOps.intake(1);
                    safeWaitSeconds(params.AutoShooterTime);
                    mechOps.feedShooter(0);
                    robot.servoFLIPPER.setPosition(params.flipper_stop);


                    follower.followPath(grabPickup2Begin, true);
                    setPathState(6);
                }
                break;
            case 6:

        /* You could check for
        - Follower State: "if(!follower.isBusy()) {}"
        - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
        - Robot Position: "if(follower.getPose().getX() > 36) {}"
        */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() || follower.getCurrentPath().getClosestPointTValue() > 0.85) {
                    /* Score Preload */
                    safeWaitSeconds(0.5);
                    //mechOps.feedShooter(params.Feeder_ON);
                    mechOps.intake(1);
                    //safeWaitSeconds(params.AutoShooterTime);
                    mechOps.feedShooter(.5);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    robot.servoFLIPPER.setPosition(params.flipper_stop);
                    follower.followPath(grabPickup2Slide, .6, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy() || follower.getCurrentPath().getClosestPointTValue() > 0.85) {
                    //turning intake on
                    robot.servoFLIPPER.setPosition(params.flipper_stop);

                    mechOps.intake(1);

                    follower.followPath(grabPickup2End, 1, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() || follower.getCurrentPath().getClosestPointTValue() > 0.85) {
                    //turning intake on
                    robot.servoFLIPPER.setPosition(params.flipper_stop);

                    mechOps.intake(1);
                    mechOps.feedShooter(.5);


                    follower.followPath(grabPickup2Shoot, true);
                    setPathState(10);
                }
                break;
            case 9:  //Skipped on Purpose!
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    //turning intake on
                    safeWaitSeconds(0.5);
                    robot.servoFLIPPER.setPosition(params.flipper_stop);
                    mechOps.intake(1);
                    mechOps.feedShooter(.5);


                    follower.followPath(scorePickup2, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    mechOps.feedShooter(0);
                    follower.followPath(scoreScore, 1, true);
                    setPathState(11);
                }
                break;

            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() || follower.getCurrentPath().getClosestPointTValue() > 0.9) {
                    /* Score Sample */
                    safeWaitSeconds(.01);
                    mechOps.feedShooter(params.Feeder_ON);
                    robot.servoFLIPPER.setPosition(params.flipper_clear);
                    safeWaitSeconds(params.AutoShooterTime);
                    mechOps.feedShooter(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    mechOps.intake(1);
                    robot.servoFLIPPER.setPosition(params.flipper_stop);

                    if (thirdline) {
                        follower.followPath(grabPickup1Begin, true);
                        setPathState(12);
                    } else if (round2) {
                        follower.followPath(endingPose, true);
                        setPathState(16);
                    } else {
                        follower.followPath(grabPickup2Begin, true);
                        round2 = true;
                        setPathState(6);
                    }
                }
                break;
            case 12:
                if (!follower.isBusy() || follower.getCurrentPath().getClosestPointTValue() > 0.85) {
                    //turning intake on
                    mechOps.intake(1);
                    mechOps.feedShooter(.5);

                    follower.followPath(grabPickup1End, true);
                    setPathState(13);
                }
                break;

            case 13:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() || follower.getCurrentPath().getClosestPointTValue() > 0.85)  {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    //follower.followPath(reversePose2, true);
                    follower.followPath(scorePickup1, true);
                    //follower.followPath(scoreScore, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy())  {
                    mechOps.feedShooter(0);
                    follower.followPath(scoreScore, 1, true);
                    setPathState(15);
                }
                break;
            case 15:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy())  {
                    /* Score Sample */
                    safeWaitSeconds(.01);
                    mechOps.feedShooter(params.Feeder_ON);
                    robot.servoFLIPPER.setPosition(params.flipper_clear);
                    mechOps.intake(1);
                    mechOps.feedShooter(1);
                    safeWaitSeconds(params.AutoShooterTime);
                    mechOps.feedShooter(0);
                    //robot.servoFLIPPER.setPosition(params.flipper_stop);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    mechOps.intake(1);
                    follower.followPath(endingPose, true);


                    setPathState(16);
                }
                break;

            case 16:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() || follower.getCurrentPath().getClosestPointTValue() > 0.85)  {
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
    public void setPathState ( int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/


    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds ( double time){
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (timer.time() < time) {
        }
    }

}
