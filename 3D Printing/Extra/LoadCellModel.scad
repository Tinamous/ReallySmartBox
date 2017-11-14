loadcellHeight = 12.75;
loadcellWidth = 12.75;
loadcellLength = 79.5;

module loadCell() {  
            
    // Bolts holes:
    // M4: 5.0
    // M4: 20.0
    // M5: 60.0
    // M5: 75.0
    // Center go from 26.75 to 52.75
    difference() {
        union() {
            cube([loadcellLength, loadcellWidth, loadcellHeight]);
            
            // strain gauge placement + goo
            translate([(loadcellLength-26)/2,0,loadcellHeight]) {
                cube([26,loadcellWidth,1]);
            }
            translate([(loadcellLength-26)/2,0,-1]) {
                cube([26,loadcellWidth,1]);
            }
            
            // Project bolts up (M4's going up)
            translate([5.0, (12.75/2),0]) {
                // Bigger top connector holes
                // to help align with top when visual testing
                cylinder(d=4, h=40);
            }
            translate([20, (12.75/2),0]) {
                cylinder(d=4, h=40);
            }
            
            // Project bolts down (M5's going down)
            translate([60, (12.75/2),-20]) {
                cylinder(d=5, h=33);
            }
            translate([75, (12.75/2),-20]) {
                cylinder(d=5, h=33);
            }
        }
        union() {
            translate([5.0, (12.75/2),0]) {
                // Bigger top connector holes
                // to help align with top when visual testing
                //#cylinder(d=4, h=40);
            }
            translate([20, (12.75/2),0]) {
                //#cylinder(d=4, h=40);
            }
            //60
            translate([60, (12.75/2),0]) {
                //cylinder(d=4, h=13);
            }
            translate([75, (12.75/2),0]) {
                //cylinder(d=4, h=13);
            }
        }
    }
}

loadCell();