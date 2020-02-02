package org.firstinspires.ftc.teamcode.auto.structurebuilder.prefab;

import org.firstinspires.ftc.teamcode.auto.structurebuilder.Layer;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.Stone;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.Structure;

public class OneByOneByNine {

    public static Structure toStructure(){
        return new Structure.StructureBuilder().addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,1,3,0)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,1,3,1)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,1,3,2)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,1,3,3)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,1,3,4)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,1,3,5)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,1,3,6)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,1,3,7)).build()
        ).addLayer(new Layer.LayerBuilder().addStone(new Stone(90,0,1,3,8)).build()).build();
    }
}
