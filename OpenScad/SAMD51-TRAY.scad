
$fn=50;

//countx, county - element count on X and Y axes
//isolx, isoly - clearance parameters
//size{x,y,z} - part size
//frmborder - whole tray border (frame)
//pheight - tray height
//center - center tray (1 - on, 0 - off)

module smdholderplate(countx, county, isolx, isoly, sizex, sizey, sizez, frmborder, pheight, center)
{
	fx = countx * sizex + (countx - 1) * isolx + 2 * frmborder;
	fy =  county * sizey + (county - 1) * isoly + 2 * frmborder;
	echo(fx);
	echo(fy);
	translate([center*(-fx/2), center*(-fy/2), 0])
	difference() {
		cube([fx, fy, pheight]);
		for(x = [1:countx]) {
			for(y = [1:county]) {
				translate([ frmborder + (x - 1) * sizex + (x -1) * isolx,
						frmborder + (y - 1) * sizey + (y -1) * isoly,
						pheight - sizez + 0.1]) cube([sizex, sizey, sizez]);
			}
		}
	}
}

smdholderplate(
    countx=24,
    county=2,
    isolx=2,
    isoly=2,
    sizex=7.2,
    sizey=7.2,
    sizez=1.4,
    frmborder=2,
    pheight=9,
    center=0);

difference(){
translate([38, 20, 0]) cube([14, 14, 2]);
translate([45, 28, 0]) cylinder(5.3, 2.65, 3);
}


difference(){
translate([170, 20, 0]) cube([14, 14, 2]);
translate([177, 28, 0]) cylinder(5.3, 2.65, 3);
}

