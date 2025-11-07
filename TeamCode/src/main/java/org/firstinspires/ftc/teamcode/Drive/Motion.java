package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

//This is a blueprint for making objects
public class Motion {

    //These are the instance variables-- when we use the constructor method to create an object whose
    //they are instance as they do not have the words static before them and because they are
    //declared in the class and not inside a method
    DcMotorEx fL, bL, fR, bR;

    OpMode opMode;

    public GoBildaPinpointDriver odo;

        //This is a constructor method
    public Motion(OpMode opMode) {
        this.opMode = opMode;

        //This is the definitions of our drive motors.
        fL = opMode.hardwareMap.get(DcMotorEx.class, "front_left_motor");
        bL = opMode.hardwareMap.get(DcMotorEx.class, "back_left_motor");
        fR = opMode.hardwareMap.get(DcMotorEx.class, "front_right_motor");
        bR = opMode.hardwareMap.get(DcMotorEx.class,"back_right_motor");

        //for mecanum drive-- positive power makes robot go forward with relative power- left motors need to be negated
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        //This tells the program we are using the encoders on the motors
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setOffsets(6, 6, DistanceUnit.INCH);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


    }
    //This is a method that will return the position of the robot on the field (instance method)
    public Pose2D getPose() {
        return odo.getPosition();
    }

    public void setPose(Pose2D pose) {
        //called the method and passed the parameter in to set the position (where the robot is)
        odo.setPosition(pose);
    }

    public void updateOdometry() {
        odo.update();
    }

    public void setDrivePower(double px, double py, double pa) {
        double pBL = px + py - pa;
        double pFL = px - py - pa;
        double pFR = px + py + pa;
        double pBR = px - py + pa;
        //power of motor -1 <= power <=1

        double max = 1;

        //assignment of the max variable and calls the math.max function that will give back the largest number
        max = Math.max(max, Math.abs(pBL));
        max = Math.max(max, Math.abs(pFL));
        max = Math.max(max, Math.abs(pFR));
        max = Math.max(max, Math.abs(pBR));

        if (max >1) {
            pBL /= max; //Java short hand for pbl = pbl/max
            pFL /= max;
            pFR /= max;
            pBR /= max;
        }

        bL.setPower(pBL);
        fL.setPower(pFL);
        fR.setPower(pFR);
        bR.setPower(pBR);


    }

}
