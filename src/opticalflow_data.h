
struct optical_flow_data{
	int h;//index to number of x in dem-map
	int w;//index to number of z in dem-map
	float vx;
	float vy;
	float vz;
	optical_flow_data() :vx(0),vy(0),vz(0){}
	
};

