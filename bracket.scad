echo(version=version());

difference() {
    color("yellow")
        translate([0, 125/2, 0])
            linear_extrude(height = 1500)
                square([3500, 125 ], center = true);
    color("blue")
        translate([-1000, 125, 1250])
            rotate([90, 90, 0])
                cylinder(h=500, r=150, center=true);
    color("blue")
        translate([1000, 125, 1250])
            rotate([90, 90, 0])
                cylinder(h=500, r=150, center=true);
}

color("red")
    translate([0,1500,0])
        linear_extrude(height = 125)
            square([5000, 3000], center = true);
            