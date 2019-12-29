package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumParallelDrive implements Runnable {

    int Angle_Degrees;
    int inches;
    double speed;
    double agl_frwd;

    int Degrees;
    double MaxSpeed;
    boolean todeg;

    boolean ismove;
    HardwareMap HW;

    public MecanumParallelDrive(HardwareMap HW) {
        this.HW = HW;
    }

    MecanumWheelDriver drive = new MecanumWheelDriver();

    public void run() {

        if (ismove) {
            //drive.moveInches(Angle_Degrees, inches, speed, agl_frwd);
        } else {
            //drive.rotate(Degrees, MaxSpeed, todeg);
        }

    }

    public void setmoveinches(int Angle_Degrees, int inches, double speed, double agl_frwd) {

        this.Angle_Degrees = Angle_Degrees;
        this.inches = inches;
        this.speed = speed;
        this.agl_frwd = agl_frwd;

        ismove = true;

    }

    public void setrotate(int Degrees, double MaxSpeed, boolean todeg) {
        this.Degrees = Degrees;
        this.MaxSpeed = MaxSpeed;
        this.todeg = todeg;

        ismove = false;
    }
}
