package org.firstinspires.ftc.teamcode.Library;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;
import java.util.ArrayList;

/*
public class ObjectFilter {
    protected static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    protected static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    protected static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public enum GoldPosition[UNKNOWN, LEFT, CENTER, RIGHT];
    class Position {
        public GoldPosition goldPosition = GoldPosition.UNKNOWN;
        int numberOfObjects = 0;
    }

    public void getRelevantObjects (List<Recognition> recognitions){
        List<Recognition> filteredRecognitions = new ArrayList<Recognition>();
        for (Recognition r : recognitions) {
            if (r.getLabel().equals(LABEL_GOLD_MINERAL)) {
                filteredRecognitions.add(r);
            }
            else if (r.getLabel().equals(LABEL_SILVER_MINERAL)){
                filteredRecognitions.add(r);
            }
        }

        List<Recognition> closestObjects = new ArrayList<Recognition>();
        Recognition r = getClosestObject(filteredRecognitions);
        closestObjects.add(r);
        filteredRecognitions.remove(r);

        r = getClosestObject(filteredRecognitions);
        closestObjects.add(r);
        filteredRecognitions.remove(r);

        r = getClosestObject(filteredRecognitions);
        closestObjects.add(r);
        filteredRecognitions.remove(r);

    }

    public Recognition getClosestObject (List<Recognition> recognitions){
        Recognition closest = null;
        for (Recognition r : recognitions) {
            if (closest == null)
                closest = r;
            else if (r.getTop() > closest.getTop())
                closest = r;
        }

        return closest;
    }

    public Position getGoldPosition (List<Recognition> recognitions){
        Position goldPosition = new Position();
        goldPosition.numberOfObjects = recognitions.size();
    }
}
*/