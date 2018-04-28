
struct index_schema{
	int nx;//index to number of x in dem-map
	int nz;//index to number of z in dem-map
	float y;
	int ny;//index to shema cluster
	index_schema() : nz(-1),nx(-1),y(-1),ny(-1){}
};

struct index_image{
	int h;//height of image
	int w;//width of image
	float y;
};

/*
struct image_point{
	int h;//height of image
	int w;//width of image
};
struct index_image{
	std::vector<image_point> at;
	std::vector<float> y;
};
*/
