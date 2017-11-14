$fn=160;
d=20;

difference() {
    union() {
        sphere (d=d);
    }
    union() {
        translate([-d/2, -d/2, -d]) {
            cube(d);
        }
        translate([0,0,-0.01]) {
            cylinder(d=5.8, h=d/2+0.01);
        }
        
        translate([0,0,4]) {
            #cylinder(d=8.4, h=d/2+0.01,$fn=6);
        }
    }
}