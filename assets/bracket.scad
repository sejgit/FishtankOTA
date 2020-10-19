echo(version=version());
include <Round-Anything/minkowskiround.scad>


difference() {
    color("yellow")
        translate([0, 125, 1750/2])
                rotate([90, 0, 0])
            linear_extrude(height = 125)
                round2d(200,0)square([3500, 1750], center = true);
    color("blue")
        translate([-1250, 125, 1250])
            rotate([90, 90, 0])
                cylinder(h=500, r=150, center=true);
    color("blue")
        translate([1250, 125, 1250])
            rotate([90, 90, 0])
                cylinder(h=500, r=150, center=true);
}

color("red")
        translate([0,1500,0])
            linear_extrude(height = 125)
                round2d(200,0)square([5000, 3000], center = true);

                
            