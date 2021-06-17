$fn=200;
X = 15;
Y = 20;
Z = 15;

slot_x = 100;
slot_y = 3;
slot_z= 15;

module base() {
   difference() { cube([X, Y, Z], center = true);
 translate([0, 0, 4]) 
    cube([slot_x, slot_y, slot_z], center = true);

  }
}
//base();

module switch_support() {
switch_len=10;
    
switch_diam=16+1;
    
difference(){
union(){
 cube([1.6*switch_diam/2, 1.4*switch_diam, switch_len], center = true);
translate([switch_diam/3, 0, 0])cylinder(h=switch_len,r=1.4*switch_diam/2, center = true);
}
translate([switch_diam/3, 0, 0])cylinder(h=1.2*switch_len,r=1*switch_diam/2, center = true);

}
}
