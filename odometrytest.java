package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous(group = "test")
public class odometrytest extends LinearOpMode {
    Pose2d myPose = new Pose2d(10,-5,Math.toRadians(90));
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory myTrejectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();
        waitForStart();

        if(isStopRequested()) return;


        drive.followTrajectory(myTrejectory);
    }

}
