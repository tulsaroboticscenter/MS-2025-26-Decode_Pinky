
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

@Autonomous(name = "Art3TowerRED", group = "Examples")
public class Art3TowerP extends LinearOpMode {

    private HWProfile2 robot = new HWProfile2();
    public final static MSParams params = new MSParams();
    private LinearOpMode myOpMode = this;
//    public HardwareMap hwmap;
    private MSMechOps mechOps;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean Blue;
    private boolean NoGateClear;
    private int pathState;

    private final Pose startPose = new Pose(130, 110.5, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(85, 120, Math.toRadians(0));// Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose PrescorePose = new Pose(85, 130, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose endPose = new Pose(86, 136, Math.toRadians(0)); //  End Position of the Robot
    private final Pose BluestartPose = new Pose(15.5, 110.5, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose BluescorePose = new Pose(60.5, 136, Math.toRadians(180));// Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose BluePrescorePose = new Pose(60, 135, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose BlueendPose = new Pose(60.5, 65, Math.toRadians(180)); //  End Position of the Robot
    //private Path scorePreload;
    private PathChain scorePreload,endingPose,scoreScore,BscorePreload,Bendpose,BscoreScore;

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
        Blue = false;


        while (!isStarted() && !isStopRequested()) {
            // Check for gamepad A button press
            if (gamepad1.aWasPressed()) {
                Blue = !Blue;
            }


            // Update telemetry
            telemetry.addData("Status", "Initialized - This dose not affect color RED!!!!");
            telemetry.addData("AUTO?", Blue ? "Red" : "Blue");


            telemetry.addLine("Initialization is complete");
            telemetry.addLine("Press Start to Play");
            telemetry.update();
            sleep (50);
        }
        //waitForStart();
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
                .addPath(new BezierCurve(startPose,new Pose(89,118),scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

/* Here is an example for Constant Interpolation
scorePreload.setConstantInterpolation(startPose.getHeading()); */

        scoreScore = follower.pathBuilder()
                .addPath(new BezierLine(PrescorePose, scorePose))
                .setLinearHeadingInterpolation(PrescorePose.getHeading(),scorePose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        endingPose = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();

        BscorePreload =follower.pathBuilder()
                .addPath(new BezierCurve(BluestartPose,new Pose(60,120),BluescorePose))
                .setLinearHeadingInterpolation(BluestartPose.getHeading(), BluescorePose.getHeading())
                .build();

        BscoreScore = follower.pathBuilder()
                .addPath(new BezierLine(BluePrescorePose, BluescorePose))
                .setLinearHeadingInterpolation(BluePrescorePose.getHeading(),BluescorePose.getHeading())
                .build();

        Bendpose = follower.pathBuilder()
                .addPath(new BezierLine(BluescorePose, BlueendPose))
                .setLinearHeadingInterpolation(BluescorePose.getHeading(), BlueendPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                telemetry.addLine("calling ShooterControl");
                telemetry.update();
                safeWaitSeconds(.5);
                mechOps.intake(1);
                mechOps.shooterControl(params.ShootAutoTower);

                follower.followPath(scorePreload, .75, true);
                follower.update();

                setPathState(2);
                break;
            case 2:

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
                   // follower.followPath(grabPickup1Begin,true);
                    setPathState(3);
                }
                break;

            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    safeWaitSeconds(.01);
                    //robot.servoFLIPPER.setPosition(params.flipper_stop);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(endingPose, true);
                    setPathState(4);
                }
                break;
            case 4:
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
        //startingPose = follower.getPose()
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
