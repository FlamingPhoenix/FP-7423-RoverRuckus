package org.firstinspires.ftc.teamcode.MyClass;

public class MineralPositionViewModel {

    public class MineralPosition {
        public float angle;
        public float distance;
    }

    public MineralPosition left;
    public MineralPosition center;
    public MineralPosition right;

    public MineralPositionViewModel()
    {
        left = new MineralPosition();
        center = new MineralPosition();
        right = new MineralPosition();
    }
}
