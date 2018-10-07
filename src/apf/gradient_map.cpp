#include<apf.h>

float APF::get_grad_1(float& x0,float& x1,float& delta)
{
	return (x1-x0)/delta;
}
float APF::get_grad_2(float& x0,float& x1,float& delta)
{
	return (x1-x0)/(2*delta);
}
void APF::create_grad_map(void)
{
	//map size
	int W=map_wi;
	int H=map_hi;
	//gradient
	float grad_x;
	float grad_y;
	int delta=1;
	float delta_cell=delta*reso;
	//
	int ch_p = pot_map.channels();
	int ch_gr0 = grad_map[0].channels();
	int ch_gr1 = grad_map[1].channels();
	
	for(int h=0;h<H;h++){
		float *ppot = pot_map.ptr<float>(h);
		float *ppot_pd = pot_map.ptr<float>(h+delta);
		float *ppot_md = pot_map.ptr<float>(h-delta);
		float *pgrad0 = grad_map[0].ptr<float>(h);
		float *pgrad1 = grad_map[1].ptr<float>(h);
		for(int w=0;w<W;w++){
			if(h==0)
			{
				grad_y=	-get_grad_1(ppot_pd[w*ch_p],ppot[w*ch_p],delta_cell);
			}
			else if(h==H-delta)
			{
				grad_y=	-get_grad_1(ppot[w*ch_p],ppot_md[w*ch_p],delta_cell);
			}
			else{
				grad_y=	-get_grad_2(ppot_pd[w*ch_p],ppot_md[w*ch_p],delta_cell);
			}
			if(w==0)
			{
				grad_x=	get_grad_1(ppot[(w+delta)*ch_p],ppot[w*ch_p],delta_cell);
			}
			else if(w==W-delta)
			{
				grad_x=get_grad_1(ppot[w*ch_p],ppot[(w-delta)*ch_p],delta_cell);
			}
			else
			{
				grad_x=get_grad_2(ppot[(w+delta)*ch_p],ppot[(w-delta)*ch_p],delta_cell);
			}
			pgrad0[(w)*ch_gr0]=grad_x;
			pgrad1[(w)*ch_gr1]=grad_y;
			
		}
	}
}

