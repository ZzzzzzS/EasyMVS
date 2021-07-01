#pragma once
#include <vector>
class BasicData
{
public:
	BasicData();
	std::vector<BasicPoint> Points;
};

struct BasicPoint
{
	union
	{
		float data[4];
		struct 
		{
			float u_L;
			float v_L;
			float u_R;
			float v_R;
		};
		struct
		{
			float x;
			float y;
			float z;
		};
		struct
		{
			float p1;
			float p2;
			float p3;
		};
	};
	
	uchar R_L;
	uchar G_L;
	uchar B_L;
	uchar R_R;
	uchar G_R;
	uchar B_R;

	float Error_a;
};