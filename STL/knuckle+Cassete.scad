// tool #1 desired point
//color("black")
//{
//translate([140, 130, -40])
//cylinder(h=100, r1=10, r2=1, center=false);
//}

// tool #2 desired point
//color("black")
//{
//translate([-80, 250, -40]) rotate([0, 0, 20])
//{
//	cylinder(h=100, r1=10, r2=1, center=false);
//	rotate([0, 90, 0]) 	cylinder(h=100, r1=10, r2=1, center=false);
//	rotate([-90, 0, 0]) 	cylinder(h=100, r1=10, r2=1, center=false);
//}
//}

rotate([0, 0, -90])
translate([0,-250,45])
{
rotate([-90, 0, 0])
{
	rotate([0, -20, 0])
	{
		translate([-6600,-330,0])
		{
			import("Knuckle_paaty_printattavaksi.STL");
		}
	}
}

color("red")
{
	translate([320, 120, 50])
	{
		rotate([0, 180, 20])
		{
			import("Cassette_v2_assembly.STL");
		}
	}
}

color("green")
{
	translate([-13.5, 71, 10])
	{
		difference()
		{
			cylinder(h=80, r1=40, r2=40, center=true);
			cylinder(h=200, r1=15, r2=15, center=true);
		}
	}
}

color("green")
{
	translate([-130, 390, -30])
	{
		difference()
		{
			cylinder(h=3, r1=40, r2=40, center=true);
			cylinder(h=4, r1=20, r2=20, center=true, $fn=6);
		}
	}
}

color("green")
{
	translate([147, 370, -20])
	{
		difference()
		{
			cylinder(h=3, r1=40, r2=40, center=true);
			cylinder(h=4, r1=20, r2=20, center=true, $fn=6);
		}
	}
}

color("blue")
{

	translate([20, 462, -35])
	{
		difference()
		{
		rotate([0,0,-5])
		{
			cube([38,65,30], true);
		}
		translate([0,-12,0])
		{
			cylinder(h=100, r1=10, r2=10, center=true);
		}
}

	}	
}
}