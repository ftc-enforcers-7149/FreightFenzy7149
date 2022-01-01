/*
package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Bounds;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Point;

import java.util.ArrayList;
import java.util.Arrays;

public class LayeredBounds extends Bounds {

    ArrayList<Bounds> holes, fills;

    public LayeredBounds() { }

    public void addHoles(Bounds... holes) {
        this.addHoles((ArrayList<Bounds>) Arrays.asList(holes));
    }

    public void addHoles(ArrayList<Bounds> holes) {
        this.holes = holes;
    }

    public void addFills(Bounds... fills) {
        this.addFills((ArrayList<Bounds>) Arrays.asList(fills));
    }

    public void addFills(ArrayList<Bounds> fills) {
        this.fills = fills;
    }

    @Override
    public boolean contains(Point p) {
        for(int i = 0; i < holes.size(); i++) {
            if(holes.get(i).contains(p)) return false;
        }
        for(int i = 0; i < fills.size(); i++) {
            if(!holes.get(i).contains(p)) return false;
        }
        return true;
    }

}
*/
