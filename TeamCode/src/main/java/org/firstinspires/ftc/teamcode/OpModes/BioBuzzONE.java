
package org.firstinspires.ftc.teamcode.OpModes; // make sure this aligns with class location

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.MSParams;
import org.firstinspires.ftc.teamcode.Libs.MSMechOps;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BioBuzz One", group = "Examples")
public class BioBuzzONE extends LinearOpMode {

    private HWProfile2 robot = new HWProfile2();
    public final static MSParams params = new MSParams();
    private LinearOpMode myOpMode = this;
//    public HardwareMap hwmap;
    private MSMechOps mechOps;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

//    private final Pose startPose = new Pose(58, 8, Math.toRadians(90)); // needs a new angle Start Pose of our robot.
//    private final Pose scorePose = new Pose(80, 15, Math.toRadians(45)); // needs a new angleScoring Pose of our robot. It is facing the goal at a 135 degree angle.
//    private final Pose PrescorePose = new Pose(80, 20, Math.toRadians(45)); // needs a new angle Scoring Pose22 of our robot. It is facing the goal at a 135 degree angle.
//    private final Pose pickup1PoseEnd = new Pose(129, 82.5, Math.toRadians(-180)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose pickup1PoseBegin= new Pose(104, 82.5, Math.toRadians(-180));
//    private final Pose pickup2PoseBegin = new Pose(104, 59, Math.toRadians(-180)); // Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose pickup2PoseEnd = new Pose(129, 59, Math.toRadians(-180)); // Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose pickup3PoseBegin = new Pose(104, 36, Math.toRadians(-180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
//    private final Pose pickup3PoseEnd = new Pose(129, 36, Math.toRadians(-180)); // 180 PedroRedTowerLowest (Third Set) of Artifacts from the Spike Mark.
//    private final Pose endPose = new Pose(94, 53, Math.toRadians(45)); // 135 End Position of the Robot
    private final Pose start = new Pose (56, 8, Math.toRadians(90));
    private final Pose endStartPath = new Pose (9, 22.5877, Math.toRadians(270));
    private final Pose startPathControl1 = new Pose (60, 45, Math.toRadians(0));
    private final Pose pickupPathStart = new Pose (8.0066, 22.5877, Math.toRadians(270));
    private final Pose pickupPath = new Pose (8.5, 10, Math.toRadians(270));
    private final Pose shoot2Start = new Pose (6.3344, 8.8096, Math.toRadians(270));
    private final Pose shoot2 = new Pose (55.0538, 126.3602, Math.toRadians(270));
    private final Pose shoot2Control1 = new Pose (25.356, 101.6573, Math.toRadians(0));
    private final Pose park = new Pose (10.6209, 104.1755, Math.toRadians (15.3351));


    private Path scorePreload;
    private PathChain startPath, pickupLine, driveToShoot2,driveToPark;
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



        follower = Constants.createFollower(hardwareMap);
        telemetry.addLine("Follower is initialized!!!");
        telemetry.update();
        sleep(500);

        buildPaths();
        telemetry.addLine("BuildPaths is initialized!!!");
        telemetry.update();
        sleep(500);

        follower.setStartingPose(start);
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
//public Path startPath() {
//        return Paths.curve(start, startPathControl1, startPath).linear(start, startPath);
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
       // scorePreload = new Path(new BezierLine(startPose, scorePose));
       // scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

/* Here is an example for Constant Interpolation
scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        startPath = follower.pathBuilder()
                .addPath(new BezierCurve(start,startPathControl1,endStartPath))
                .setLinearHeadingInterpolation(start.getHeading(), endStartPath.getHeading())
                .build();

        pickupLine = follower.pathBuilder()
                .addPath(new BezierLine(endStartPath, pickupPath))
                .setLinearHeadingInterpolation(endStartPath.getHeading(), pickupPath.getHeading())

                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        driveToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickupPath,shoot2Control1,shoot2))
                .setLinearHeadingInterpolation(pickupPath.getHeading(),shoot2.getHeading())
                .build();

        driveToPark = follower.pathBuilder()
                .addPath(new BezierLine(shoot2, park))
                .setLinearHeadingInterpolation(shoot2.getHeading(),park.getHeading())
                .build();


        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line.
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
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line.
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseBegin, PrescorePose))
                .setLinearHeadingInterpolation(pickup2PoseBegin.getHeading(), PrescorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line.
        grabPickup3Begin = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3PoseBegin))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3PoseBegin.getHeading())
                .build();
        grabPickup3End = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseBegin, pickup3PoseEnd))
                .setLinearHeadingInterpolation(pickup3PoseBegin.getHeading(), pickup3PoseEnd.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line.
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseEnd, PrescorePose))
                .setLinearHeadingInterpolation(pickup3PoseEnd.getHeading(), PrescorePose.getHeading())
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line.
        endingPose = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build(); */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                telemetry.addLine("calling ShooterControl");
                telemetry.update();
                mechOps.shooterControl(1150);

                //follower.followPath(scorePreload, true);
                //follower.update();

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
                    safeWaitSeconds(1);
                    mechOps.feedShooter(.9);
                    mechOps.intake(1);
                    safeWaitSeconds(2.5);
                    mechOps.feedShooter(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(startPath,true);
                    setPathState(2);
                }
                break;

            case 2:
                    if (!follower.isBusy()) {
                        //turning intake on
                        mechOps.intake(1);

                        follower.followPath(pickupLine, true);
                        setPathState(3);
                    }
                    break;

            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(driveToShoot2, true);

                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    safeWaitSeconds(.01);
                    mechOps.feedShooter(.9);
                    safeWaitSeconds(2.5);
                    mechOps.feedShooter(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    mechOps.intake(1);
                    follower.followPath(driveToPark, true);

                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                    setPathState(10);
                }
                break;
//            case 6:
//                if (!follower.isBusy()) {
//
//                    follower.followPath(scoreScore, true);
//                    setPathState(7);
//                }
//            case 7:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
//                    /* Score Sample */
//                    safeWaitSeconds(.01);
//                    mechOps.feedShooter(.9);
//                    mechOps.intake(1);
//                    safeWaitSeconds(2.5);
//                    mechOps.feedShooter(0);
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    mechOps.intake(1);
//                    follower.followPath(grabPickup3Begin, true);
//                    follower.followPath(grabPickup3End, true);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if (!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup3, true);
//                    follower.followPath(scoreScore, true);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if (!follower.isBusy()) {
//                    /* Grab Sample */
//                    safeWaitSeconds(.01);
//                    mechOps.feedShooter(.9);
//                    mechOps.intake(1);
//                    safeWaitSeconds(2.5);
//                    mechOps.feedShooter(0);
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(endingPose, true);
//                    setPathState(10);
//                }
//                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                   mechOps.intake(0);
                   mechOps.feedShooter(0);
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
