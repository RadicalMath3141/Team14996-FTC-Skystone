package org.firstinspires.ftc.teamcode.auto.structurebuilder;

public class StructureConstructor {

    private Structure structure;

    public StructureConstructor(Structure structure){
        this.structure = structure;
    }

    public double getCurrentHeight(){
        if(structure.currentLayer().getCurrentStone() != null){
            return stoneZCoordToElevatorHeight(structure.currentLayer().getCurrentStone().getZCoord());
        }
        return 0;
    }

    public int getCurrentLayerNumber(){
        return structure.currentLayerNumber();
    }

    public double getNextHeight(){
        if(structure.currentLayer().nextStone()){
            return stoneZCoordToElevatorHeight(structure.currentLayer().getCurrentStone().getZCoord());
        } else if(structure.nextLayer()){
            return stoneZCoordToElevatorHeight(structure.currentLayer().getCurrentStone().getZCoord());
        }
        return 0;
    }

    public double getPreviousHeight(){
        if(structure.previousLayer().getCurrentStone() != null){
            return stoneZCoordToElevatorHeight(structure.currentLayer().getCurrentStone().getZCoord());
        }
        return 0;
    }

    public boolean ifCanGoHigher(){
        //I am subtracting one due to zero indexing
        if(structure.numLayers() - 1 - structure.currentLayerNumber() > 0){
            return true;
        } else {
            return false;
        }
    }

    public double stoneZCoordToElevatorHeight(int zCoord){
        return zCoord * 4 + 3.25;
    }

}
