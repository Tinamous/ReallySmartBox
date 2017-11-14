// Sits between the base sheet and the load cell to raise the cell slightly and provide a side on branding message :-)


paddingHeight = 8; // 8mm max for 3mm acrylic with M5x12
loadCellDepth = 12.75;
// Depth is 26mm (offset of screws from front of base panel)
// plus the other half of the load cell
blockDepth =  26 + (loadCellDepth/2); // loadCellDepth + 2 + 6; // 13mm load cell, 6mm riser block depth and 2mm space

// Total length of the load cell padding.
//overallWidth = 120; //
overallWidth = 190; //

// Load cell first hole is 70mm in from the edge. - not a drilled hole.
// Load acual cell first hole is 126.2mm in from the edge.
// Load cell runs form 65mm to 145mm (80mm).

// We are 20mm wither side of the load cell.

rotate([0,0,-90]) {
    translate([-26,68.5+35,8]) {
    
       %import("./Extra/LoadCellModel.stl");
    }
}


difference() {
    union() {
        // Basic floor point for the thing.
        translate([10,0,0]) {
            cube([overallWidth-20, blockDepth, 2]);
        }
        
        // Front riser for text
        // TODO: Make height dynamic based on paddingHeight
        translate([0,2,0]) {
            cube([overallWidth, 2, 27]);
        }
        
        translate([1,2.1,6]) {
            rotate([90,0,0]) {
                linear_extrude(2) {
                    text("Really Smart Box", size=18);
                }
            }
        }
        
        // Padder is 26mm wide
        // 20mm padding either end of the section.
        translate([overallWidth - 81,blockDepth-13,0]) {
            // 6mm riser block with 2mm space
            #cube([26,13, paddingHeight]);
        }
    }
    union() {
        // Hole is 26mm from front edge of panel
        translate([overallWidth - 81 +6.5, 26 , -0.1]) {
            //translate([2, blockDepth, -0.1]) {
            #cylinder(d=6, h=paddingHeight+0.2, $fn=60);
            
            // 15mm gap between holes
            translate([15.0, 0 ,0]) {
                #cylinder(d=6, h=paddingHeight+0.2, $fn=60);
            }
        }
        
        // Hole for an LED
        translate([overallWidth/2, 3 ,22/2]) {
            rotate([90,0,0]) {
               // #cylinder(d=5.2, h=paddingHeight+0.2, $fn=60);
            }
        }
    }
}