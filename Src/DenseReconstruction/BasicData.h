#pragma once
#include <vector>
#include <opencv.hpp>

struct BasicPoint
{
	union
	{
		float data[4] = { 0 };
		struct
		{
			float u_L;
			float v_L;
			float u_R;
			float v_R;
		};
		struct
		{
			float q1;
			float q2;
			float q3;
			float q4;
		};
		struct
		{
			float x;
			float y;
			float z;
		};
		struct
		{
			float x1;
			float x2;
			float x3;
		};
		struct
		{
			float p1;
			float p2;
			float p3;
		};
	};

	uchar R_L = 0;
	uchar G_L = 0;
	uchar B_L = 0;
	uchar R_R = 0;
	uchar G_R = 0;
	uchar B_R = 0;

	float Error_a;
};

class BasicData
{
public:
	BasicData();
	std::vector<BasicPoint> Points;
};

