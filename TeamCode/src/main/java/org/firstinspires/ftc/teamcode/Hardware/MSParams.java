package org.firstinspires.ftc.teamcode.Hardware;

public class MSParams {
    /*
     * Constants
     */
    public final double DRIVE_TICKS_PER_INCH = 24;
    public final double STRAFE_FACTOR = 0.9;

    //Flipper commands
    public final double flipper_clear = .6;
    public final double flipper_stop = 0.2;//Started at .4 tried .3
    public final double flipper_rev = flipper_clear;
    public final double LIFTZero = 0.3;
    public final double LIFTlifting = 0.75;
    public final double Far_Shooter = 1;
    public final double Close_Shooter = .5;
    public final double Intake_ON = 1;
    public final double Intake_Rev = -0.5;
    public final double Intake_OFF = 0;
    public final double Feeder_ON = 1; //was reaching 3000
    public final double Feeder_REV = -.5;
    public final double Feeder_OFF = 0;
    public final double ShootAutoStart = 1590; //was 1661 was 1600
    public final double ShootAutoLong = 1580; // was 1610 was1590 ignore the 1610.
    public final double ShootAutoTower = 1400;
    public final double ShootTeleNear = 1520;
    public final double ShootTeleFar = 1580;
    public final double ShootStart = 1720;
    public final double TriggerPulse = 50;
    public final double TriggerVel = 1600;
    public final double AutoShooterTime = 2;
    public final double BallAirTime = 0.35;
    public final double Hood1Close = .55;
    public final double Hood1Far = .7;
    public final double HoodMIN =.54;
    public final double HoodMAX =.66;

}
