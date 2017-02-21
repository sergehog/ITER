rotate([0, 0, 90])
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

//color("green")
//{
//	translate([-13.5, 71, -40])
//	{
//		difference()
//		{
//			cylinder(h=3, r1=40, r2=40, center=true);
//			cylinder(h=4, r1=20, r2=20, center=true);
//		}
//	}
//}

color("green")
{
	translate([-130, 390, -20])
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

	translate([20, 460, -35])
	{
		difference()
		{
		rotate([0,0,-5])
		{
			cube([40,60,30], true);
		}
		translate([0,-12,0])
		{
			cylinder(h=100, r1=8, r2=8, center=true);
		}
}

	}	
}
}