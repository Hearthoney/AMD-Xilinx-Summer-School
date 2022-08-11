#include "cordiccart2pol.h"

void cordiccart2pol(data_t x, data_t y, data_t * r,  data_t * theta)
{
#pragma HLS INTERFACE s_axilite port=x  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=y  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=r  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=theta  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=return

	para_t i;
	data_t c_x;
	data_t c_y;
	data_t tem;
	data_t theta1;

	static const double cf0[16] = { 0.785398163397448,	0.463647609000806,	0.244978663126864,
		0.124354994546761,	0.0624188099959574,	0.0312398334302683,	0.0156237286204768,
		0.00781234106010111,	0.00390623013196697,	0.00195312251647882,	0.000976562189559320,	0.000488281211194898,
		0.000244140620149362,	0.000122070311893670,	6.10351561742088e-05,	3.05175781155261e-05 };

	static const double cf1[16] = { 1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125,
    0.015625, 0.0078125, 0.00390625, 0.001953125, 0.0009765625, 0.00048828125,
    0.000244140625, 0.0001220703125, 6.103515625E-5, 3.0517578125E-5 };

	
	if( x < 0.0 )
	{
		if( y > 0.0 )
		{	
			tem = x;
			x = y;
			y = -tem;
			*theta = 1.5708;
		}
		else
		{
			tem = x;
			x = -y;
			y = tem;
			*theta = -1.5708;
		}
	}
	else
	{
		*theta = 0.0;
	}
	
	for (i = 0; i < 16; i++)
	{
	#pragma HLS LOOP_TRIPCOUNT min=1 max=16
	#pragma HLS pipeline
		if( y >= 0.0 )
		{	
			tem = cf0[i];
			c_x = x + y * cf1[i];
        	c_y = y - x * cf1[i];
		}
		else
		{
			tem = -cf0[i];
			c_x = x - y * cf1[i];
        	c_y = y + x * cf1[i];
		}
			x = c_x;
			y = c_y;
			*theta = *theta + tem;
			*r = x / 1.6468;
	}

}
