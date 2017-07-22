void HoughParabolas (Mat img, float thresh)
{
	int rows = img.rows, cols = img.cols;
	float curvatures[] = {0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007, 0.008};

	Mat accum = Mat::zeros(rows, cols, sizeof(curvatures)/sizeof(curvatures[0]));
	vector<Point> nonZero;
	findNonZero(img, nonZero);

	for(int i = 0; i < nonZero.size(); i++)
	{
		for(int j = 0; j < sizeof(curvatures)/sizeof(curvatures[0]); j++)
		{
			for(int k = 0; k < cols; i++)
			{
				float y0 = nonZero[i].y - curvatures[j]*(nonZero[i].x - k)*(nonZero[i].x - k);
				y0 = round(y0);
				if (y0 < rows && y0 >=1)
				{
					accum.at<double>(i, k, j) ++;
				}
			}
		}
	}

	Mat amax;
	dilate(accum, amax, Mat::ones(20,20,4));
	Mat accumMask  = amax > thresh;
	vector<Point3i> nonZero2;
	findNonZero(accumMask, nonZero2);
	polylines(img, nonZero2, false, Scalar(0, 0, 255), 5);
	/*int x0;
	float a0;

	for(int l = 0; l< nonZero2.size(); l++)
	{
		x0 = nonZero2[l].x;
		y0 = nonZero2[l].y;
		a0 = nonZero2[l].z;
		for(int x = 0; x < cols; x++)
		{
			int y = y0 + a0*(x - x0)*(x - x0);
			if (y <= rows && y >=1)
			{
				parabola(img, x0,y0,a0)
			}

		}
	}*/




}
