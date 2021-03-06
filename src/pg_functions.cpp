#include <pg_functions.h>

proto_functions::proto_functions()
{
	//this is the brach 3axes
	/******* common part *******/

	size_ = 15;
	silence_ = 0;
	imu_num_ = 5;
	count_filt_ = 0;

	a_acquisition_flag_ = false;
	g_acquisition_flag_ = false;
	work_flag_ = false;
	wait_flag_ = false;

	sub_acc_ = n_.subscribe("/qb_class_imu/acc", 28, &proto_functions::callAcc, this);
	sub_gyro_ = n_.subscribe("/qb_class_imu/gyro", 28, &proto_functions::callGyro, this);
	pub_fig_ = n_.advertise<std_msgs::String>("/which_finger",100);
	filt_data_ = n_.advertise<qb_interface::inertialSensorArray>("/filtered_data",100);

	tmp_acc_.resize(7,3);
	tmp_acc_.setZero(7,3);
	tmp_gyro_.resize(7,3);
	tmp_gyro_.setZero(7,3);

	tmp_acc_old_.resize(7,3);
	tmp_acc_old_.setZero(7,3);
	tmp_gyro_old_.resize(7,3);
	tmp_gyro_old_.setZero(7,3);


	/******* accelerations part *******/

	cross_value_ = 0;
	axis_ = 100; //NaN
	database_size_ = 13; //#rows

	filt_acc_.resize(7,3);
	filt_acc_.setZero(7,3);

	old_filt_acc_.resize(7,3);
	old_filt_acc_.setZero(7,3);

	// matrix for accelerations
	matrix_acc_.resize(21,size_);
	matrix_acc_.setZero(21,size_);
	single_acc_.resize(7,size_*3);
	single_acc_.setZero(7,size_*3);

	// matrix for correlation
	database_window_.resize(database_size_*3,size_);
	database_window1_.resize(database_size_,3*size_);



	/******* gyroscopes part ******/

	// matrix for gyroscopes
	matrix_gyro_x_.resize(7,size_);
	matrix_gyro_y_.resize(7,size_);
	matrix_gyro_z_.resize(7,size_);

	matrix_gyro_x_.setZero(7,size_);
	matrix_gyro_y_.setZero(7,size_);
	matrix_gyro_z_.setZero(7,size_);
	
	// build matrix to use for cross-correlation
	std::cout<<"Getting database...\n\n";
	getDatabaseWindow();

}

/*********************************************************************/
/*********************************************************************/

proto_functions::~proto_functions()
{}

/*********************************************************************/
/*********************************************************************/

void proto_functions::getDatabaseWindow()
{
	int k, h;
	std::vector<std::string> imu_name;
	std::vector<float> database_vector;
	database_vector.resize(1,size_*3);

	imu_name.resize(database_size_);
	imu_name[0] = "side_little";
	imu_name[1] = "frontal_little";
	imu_name[2] = "vertical_little";
	imu_name[3] = "frontal_ring";
	imu_name[4] = "vertical_ring";
	imu_name[5] = "frontal_middle";
	imu_name[6] = "vertical_middle";
	imu_name[7] = "side_index";
	imu_name[8] = "frontal_index";
	imu_name[9] = "vertical_index";
	imu_name[10] = "side_thumb";
	imu_name[11] = "frontal_thumb";
	imu_name[12] = "vertical_thumb";

	//loop for primitive
	for (k=0; k<database_size_; k++)
	{
		n_.getParam(imu_name[k], database_vector);

		//loading x, y, z values into three different rows
		for (h=0; h<size_; h++)
			{
				database_window_(3*k,h) = database_vector[0,3*h];		//x
				database_window_(3*k+1,h) = database_vector[0,3*h+1];	//y
				database_window_(3*k+2,h) = database_vector[0,3*h+2];	//z
			}

		for (h=0; h<size_*3; h++)
		{
			database_window1_(k,h) = database_vector[0,h];
		}
	}

	sleep(1);

	std::cout << "\n\nREADY!\n\n";

	a_acquisition_flag_ = true;
	g_acquisition_flag_ = true;

}

/*********************************************************************/
/*********************************************************************/

void proto_functions::callAcc(qb_interface::inertialSensorArray imu_acc)
{
	if (a_acquisition_flag_)
	{
		acc_ = imu_acc;
		a_acquisition_flag_ = false;
	}
}

/*********************************************************************/
/*********************************************************************/

void proto_functions::callGyro(qb_interface::inertialSensorArray imu_gyro)
{
	if (g_acquisition_flag_)
	{
		gyro_ = imu_gyro;
		g_acquisition_flag_ = false;
	}	
}

/*********************************************************************/
/*********************************************************************/

void proto_functions::waitCall()
{
	int i;
	ros::Rate r(2000);

	do
	{
		for (i=0; i<7; i++)
		{
			// old
			tmp_acc_old_(i,0) = tmp_acc_(i,0);
			tmp_acc_old_(i,1) = tmp_acc_(i,1);
			tmp_acc_old_(i,2) = tmp_acc_(i,2);

			tmp_gyro_old_(i,0) = tmp_gyro_(i,0);
			tmp_gyro_old_(i,1) = tmp_gyro_(i,1);
			tmp_gyro_old_(i,2) = tmp_gyro_(i,2);

			// new
			tmp_acc_(i,0) = acc_.m[i].x;
			tmp_acc_(i,1) = acc_.m[i].y;
			tmp_acc_(i,2) = acc_.m[i].z;

			tmp_gyro_(i,0) = gyro_.m[i].x;
			tmp_gyro_(i,1) = gyro_.m[i].y;
			tmp_gyro_(i,2) = gyro_.m[i].z;
		}

		if ( (tmp_acc_ - tmp_acc_old_).sum() != 0 )
			a_acquisition_flag_ = false;
		else
			a_acquisition_flag_ = true;

		if ( (tmp_gyro_ - tmp_gyro_old_).sum() != 0 )
			g_acquisition_flag_ = false;
		else
			g_acquisition_flag_ = true;

		ros::spinOnce();
		r.sleep();
	}
	while(a_acquisition_flag_ && g_acquisition_flag_);

	if (!a_acquisition_flag_ && !g_acquisition_flag_)
		work_flag_ = true;
}

/*********************************************************************/
/*********************************************************************/

/*###################################################################*/
/*#################<<<<<ACCELERATIONS PART>>>>>######################*/
/*###################################################################*/

/*********************************************************************/
/*********************************************************************/

void proto_functions::filterData()
{
	float f_cut_ = 5;
	float pi = 3.14159;
	float dt = 0.012;

	float RC = 1/(2*pi*f_cut_);
	float alpha = RC / (RC+dt);
	int i;

	qb_interface::inertialSensor tmp_filt;
	qb_interface::inertialSensorArray filt_;

	if (count_filt_ < 3)
	{	
	  	for (i=0; i<7; i++)
	  	{
	  		filt_acc_(i,0) = tmp_acc_(i,0);
	  		filt_acc_(i,1) = tmp_acc_(i,1);
	  		filt_acc_(i,2) = tmp_acc_(i,2);
	  	}

	count_filt_ ++;

	}

	if (count_filt_ > 2)
	{
	  	for (i=0; i<7; i++)
	  	{
	  		filt_acc_(i,0) = alpha * ( old_filt_acc_(i,0) + tmp_acc_(i,0) - tmp_acc_old_(i,0) );
	  		filt_acc_(i,1) = alpha * ( old_filt_acc_(i,1) + tmp_acc_(i,1) - tmp_acc_old_(i,1) );
	  		filt_acc_(i,2) = alpha * ( old_filt_acc_(i,2) + tmp_acc_(i,2) - tmp_acc_old_(i,2) );

	  		old_filt_acc_(i,0) = filt_acc_(i,0);
	  		old_filt_acc_(i,1) = filt_acc_(i,1);
	  		old_filt_acc_(i,2) = filt_acc_(i,2); 

	  		tmp_filt.x = filt_acc_(i,0);
	  		tmp_filt.y = filt_acc_(i,1);
	  		tmp_filt.z = filt_acc_(i,2);

	  		filt_.m.push_back(tmp_filt);
	  	}

		filt_data_.publish(filt_);

		ros::spinOnce();
	}
	

}

/*********************************************************************/
/*********************************************************************/

void proto_functions::updateAccMatrix()
{
	int i,j;
	
	for(j=0; j<size_-1; j++)	//select the column
	{
		for (i=0; i<21; i++)	//select the imu axis
		{
			matrix_acc_(i,j) = matrix_acc_(i,j+1);
		}
	}

	for (i=0; i<7; i++)
	{
		for (j=0; j<size_-1; j++)
		{
			single_acc_(i,3*j) = single_acc_(i,3*j+3);
			single_acc_(i,3*j+1) = single_acc_(i,3*j+4);
			single_acc_(i,3*j+2) = single_acc_(i,3*j+5);
		}
	}
}

/*********************************************************************/
/*********************************************************************/

void proto_functions::loadAcc()
{
	int i;

  for(i=0; i<7; i++)
  {
  	matrix_acc_(3*i,size_-1) = filt_acc_(i,0);
  	matrix_acc_(3*i+1,size_-1) = filt_acc_(i,1);
  	matrix_acc_(3*i+2,size_-1) = filt_acc_(i,2);

  	single_acc_(i,size_-3) = filt_acc_(i,0);
  	single_acc_(i,size_-2) = filt_acc_(i,1);
  	single_acc_(i,size_-1) = filt_acc_(i,2);
  }
}

/*********************************************************************/
/*********************************************************************/

bool proto_functions::AccControl()
{
	int i;
	int hit = 100;	//NaN
	float trade = 0.1;	
	float value = 0;
	std::vector<float> imu_value;
	imu_value.resize(imu_num_);

	//taking the maximum value for each imu
	for (i=0; i<imu_num_; i++)
		imu_value[i] = checkMaxAcc(i); 

	//checking the maximum
	for (i=0; i<imu_num_; i++)
	{
		if (value < imu_value[i])
		{
			value = imu_value[i];
			hit = i;
		}
	}

	if (hit == 100)
		return false; 	

	//control on lower limit
	if (imu_value[hit] > trade)
	{
		hit_imu_ = hit;
		return true;
	}
	else
		return false;

}


/*********************************************************************/
/*********************************************************************/

float proto_functions::checkMaxAcc(int imu)
{
	int i;
	float max_value = 0;
	std::vector<float> tmp_value;
	tmp_value.resize(3);
	// int axis;
	float max1, max2;

	for (i=0; i<3; i++)
		tmp_value[i] = 0;


	//the recorded pic is in position number 4
	for (i=3; i<4; i++)
	{
		if(fabs(matrix_acc_(3*imu,i)) > fabs(tmp_value[0]))		//max on x
			tmp_value[0] = matrix_acc_(3*imu,i);

		if(fabs(matrix_acc_(3*imu+1,i)) > fabs(tmp_value[1]))		//max on y
			tmp_value[1] = matrix_acc_(3*imu+1,i);

		if(fabs(matrix_acc_(3*imu+2,i)) > fabs(tmp_value[2]))		//max on z
			tmp_value[2] = matrix_acc_(3*imu+2,i);
	}

	/*
	max1 = std::max(fabs(tmp_value[0]),fabs(tmp_value[1]));
	max2 = std::max(fabs(tmp_value[1]),fabs(tmp_value[2]));
	max_value = std::max(fabs(max1),fabs(max2));

	return max_value;
	*/


	if (fabs(tmp_value[0]) > fabs(tmp_value[1]) && fabs(tmp_value[0]) > fabs(tmp_value[2]))
	{
		max_value = fabs(tmp_value[0]);
		// direction_ = 0;
		return max_value;
	}
	else if (fabs(tmp_value[1]) > fabs(tmp_value[0]) && fabs(tmp_value[1]) > fabs(tmp_value[2]))
	{
		max_value = fabs(tmp_value[1]);
		// direction_ = 1;
		return max_value;
	}
	else if (fabs(tmp_value[2]) > fabs(tmp_value[0]) && fabs(tmp_value[2]) > fabs(tmp_value[1]))
	{
		max_value = fabs(tmp_value[2]);
		// direction_ = 2;
		return max_value;
	}

}

/*********************************************************************/
/*********************************************************************/

float proto_functions::crossCorrelation(int axis, int d)
{
	//axis is referred to matrix_acc_, d to database_window_
   int i;
   int n = 15;	//window size to do xcorr
   int s = 0;	//start
   float r;
   float mean_x, mean_y, sum_x, sum_y, num_xy, denom;

   mean_x = 0;
   mean_y = 0;

   num_xy=0;

   sum_x = 0;
   sum_y = 0;

   for (i=s; i<n; i++) 
   {
   		mean_x = mean_x + matrix_acc_(axis,i);
   		mean_y = mean_y + database_window_(d,i);
   }

   mean_x = mean_x / (n-s);
   mean_y = mean_y / (n-s);

   //computing numerator and denominator
   for (i=s; i<n; i++)
   {
   		num_xy = num_xy + (matrix_acc_(axis,i) - mean_x) * (database_window_(d,i) - mean_y);
   		sum_x = sum_x + (matrix_acc_(axis,i) - mean_x) * (matrix_acc_(axis,i) - mean_x);
      	sum_y = sum_y + (database_window_(d,i) - mean_y) * (database_window_(d,i) - mean_y);
    }

    denom = sqrt(sum_x*sum_y);

   /*computing correlation value*/
    r = num_xy / denom;

    return r;
}

/*********************************************************************/
/*********************************************************************/

void proto_functions::possibleFinger()
{
	float xcorr_lower_limit = 0.50;
	std::vector<float> xcv;
	xcv.resize(1,3);

	switch(hit_imu_)
	{
		case 0:	//case little
		{
			switch(axis_)
			{
				case 0:	//case axis
				{
					xcv[0,0] = crossCorrelation(0,0);	//x axis
					xcv[0,1] = crossCorrelation(1,1);	//y axis
					xcv[0,2] = crossCorrelation(2,2);	//z axis
					
					cross_value_ = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					if (cross_value_ >= xcorr_lower_limit)
						answer(0);
					
					break;
				}
				case 1:	//case axis
				{			
					xcv[0,0] = crossCorrelation(0,3);	//x axis
					xcv[0,1] = crossCorrelation(1,4);	//y axis
					xcv[0,2] = crossCorrelation(2,5);	//z axis

					cross_value_ = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					if (cross_value_ >= xcorr_lower_limit)
						answer(1);
					
					break;
				}
				case 2:	//case axis
				{
					float cr1, cr2;

					// frontal
					xcv[0,0] = crossCorrelation(0,3);	//x axis
					xcv[0,1] = crossCorrelation(1,4);	//y axis
					xcv[0,2] = crossCorrelation(2,5);	//z axis

					cr1 = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					// vertical
					xcv[0,0] = crossCorrelation(0,6);	//x axis
					xcv[0,1] = crossCorrelation(1,7);	//y axis
					xcv[0,2] = crossCorrelation(2,8);	//z axis

					cr2 = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));
										
					cross_value_ = std::max(cr1,cr2);

					if (cr1 > cr2 && cr1 >= xcorr_lower_limit)
						answer(1);
					else if(cr2 > cr1 && cr2 >= xcorr_lower_limit)
						answer(2);
				}
			}
			break;
		}

		case 1:	//case ring
		{
			switch(axis_)
			{
				case 0:	//case axis
				{
					// std::cout << "\nMaybe something else was hit..\n";
					break;
				}
				case 1:	//case axis
				{

					xcv[0,0] = crossCorrelation(3,9);	//x axis
					xcv[0,1] = crossCorrelation(4,10);	//y axis
					xcv[0,2] = crossCorrelation(5,11);	//z axis

					cross_value_ = std::max(std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					if (cross_value_ >= xcorr_lower_limit)
						answer(3);
					
					break;
				}
				case 2:	//case axis
				{
					float cr1, cr2;

					//frontal
					xcv[0,0] = crossCorrelation(3,9);	//x axis
					xcv[0,1] = crossCorrelation(4,10);	//y axis
					xcv[0,2] = crossCorrelation(5,11);	//z axis

					cr1 = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					//vertical
					xcv[0,0] = crossCorrelation(3,12);	//x axis
					xcv[0,1] = crossCorrelation(4,13);	//y axis
					xcv[0,2] = crossCorrelation(5,14);	//z axis

					cr2 = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					cross_value_ = std::max(cr1,cr2);

					if (cr1 > cr2 && cr1 >= xcorr_lower_limit)
						answer(3);
					else if(cr2 > cr1 && cr2 >= xcorr_lower_limit)
						answer(4);

					break;
				}
			}
			break;
		}

		case 2:	//case middle
		{
			switch(axis_)
			{
				case 0:	//case axis
				{
					// std::cout << "\nMaybe something else was hit..\n";
					break;
				}
				case 1:	//case axis
				{
					xcv[0,0] = crossCorrelation(6,15);	//x axis
					xcv[0,1] = crossCorrelation(7,16);	//y axis
					xcv[0,2] = crossCorrelation(8,17);	//z axis

					cross_value_ = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					if (cross_value_ >= xcorr_lower_limit)
						answer(5);
					
					break;
				}
				case 2:	//case axis
				{	
					float cr1, cr2;

					xcv[0,0] = crossCorrelation(6,15);	//x axis
					xcv[0,1] = crossCorrelation(7,16);	//y axis
					xcv[0,2] = crossCorrelation(8,17);	//z axis

					cr1 = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					xcv[0,0] = crossCorrelation(6,18);	//x axis
					xcv[0,1] = crossCorrelation(7,19);	//y axis
					xcv[0,2] = crossCorrelation(8,20);	//z axis

					cr2 = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					cross_value_ = std::max(cr1,cr2);

					if (cr1 > cr2 && cr1 >= xcorr_lower_limit)
						answer(5);
					else if(cr2 > cr1 && cr2 >= xcorr_lower_limit)
						answer(6);

					break;	
				}
			}
			break;
		}

		case 3:	//case index
		{
			switch(axis_)
			{
				case 0:	//case axis
				{
					xcv[0,0] = crossCorrelation(9,21);	//x axis
					xcv[0,1] = crossCorrelation(10,22);	//y axis
					xcv[0,2] = crossCorrelation(11,23);	//z axis

					cross_value_ = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					if (cross_value_ >= xcorr_lower_limit)
						answer(7);
					
					break;
				}
				case 1:	//case axis
				{
					xcv[0,0] = crossCorrelation(9,24);	//x axis
					xcv[0,1] = crossCorrelation(10,25);	//y axis
					xcv[0,2] = crossCorrelation(11,26);	//z axis

					cross_value_ = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					if (cross_value_ >= xcorr_lower_limit)
						answer(8);
					
					break;
				}
				case 2:	//case axis
				{
					float cr1, cr2;

					// frontal
					xcv[0,0] = crossCorrelation(9,24);	//x axis
					xcv[0,1] = crossCorrelation(10,25);	//y axis
					xcv[0,2] = crossCorrelation(11,26);	//z axis

					cr1 = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					// vertical
					xcv[0,0] = crossCorrelation(9,27);	//x axis
					xcv[0,1] = crossCorrelation(10,28);	//y axis
					xcv[0,2] = crossCorrelation(11,29);	//z axis

					cr2 = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					cross_value_ = std::max(cr1, cr2);

					if (cr1 > cr2 && cr1 >= xcorr_lower_limit)
						answer(8);
					else if (cr2 > cr1 && cr2 >= xcorr_lower_limit)
						answer(9);
					
					break;	
				}
			}
			break;
		}

		case 4:	//case thumb
		{
			switch(axis_)
			{

				case 0:	//case axis
				{
					xcv[0,0] = crossCorrelation(12,30);	//x axis
					xcv[0,1] = crossCorrelation(13,31);	//y axis
					xcv[0,2] = crossCorrelation(14,32);	//z axis

					cross_value_ = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					if (cross_value_ >= xcorr_lower_limit)
						answer(10);
					
					break;
				}
				case 1:	//case axis
				{
					
					xcv[0,0] = crossCorrelation(12,33);	//x axis
					xcv[0,1] = crossCorrelation(13,34);	//y axis
					xcv[0,2] = crossCorrelation(14,35);	//z axis

					cross_value_ = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					if (cross_value_ >= xcorr_lower_limit)
						answer(11);
					
					break;
				}
				case 2:	//case axis
				{
					xcv[0,0] = crossCorrelation(12,36);	//x axis
					xcv[0,1] = crossCorrelation(13,37);	//y axis
					xcv[0,2] = crossCorrelation(14,38);	//z axis

					cross_value_ = std::max( std::max(xcv[0,0],xcv[0,1]), std::max(xcv[0,0],xcv[0,2]));

					if (cross_value_ >= xcorr_lower_limit)
						answer(12);
					
					break;
				}
			}
			break;
		}

		default:
		{
			break;
		}

	}
}

/*********************************************************************/
/*********************************************************************/

void proto_functions::answer(int direction)
{
	std_msgs::String finger;

	std::vector<std::string> f;
	f.resize(database_size_+1);
	f[0] = "side_little";
	f[1] = "frontal_little";
	f[2] = "vertical_little";
	f[3] = "frontal_ring";
	f[4] = "vertical_ring";
	f[5] = "frontal_middle";
	f[6] = "vertical_middle";
	f[7] = "side_index";
	f[8] = "frontal_index";
	f[9] = "vertical_index";
	f[10] = "vertical_thumb";
	f[11] = "frontal_thumb";
	f[12] = "side_thumb";
	f[13] = "default";

	wait_flag_ = true;

	std::cout << "Hit imu number\t" << hit_imu_ << std::endl;
	std::cout << "on\t" << f[direction] << std::endl;
	std::cout << "cross value equal to\t" << cross_value_ << std::endl;
	
	finger.data = f[direction];
	pub_fig_.publish(finger);
	
	cross_value_ = 0;

	sleep(1);
	//to avoid continuing grasp
	direction = 13;
	finger.data = f[direction];
	pub_fig_.publish(finger);
}


/*********************************************************************/
/*********************************************************************/


/*###################################################################*/
/*#################<<<<<GYROSCOPES PART>>>>>######################*/
/*###################################################################*/


/*********************************************************************/
/*********************************************************************/

void proto_functions::updateGyroMatrix()
{
	int i,j;

	for(j=0; j<size_-1; j++)	//select the column
	{
		for (i=0; i<7; i++)		//select the imu
		{
			matrix_gyro_x_(i,j) = matrix_gyro_x_(i,j+1);
			matrix_gyro_y_(i,j) = matrix_gyro_y_(i,j+1);
			matrix_gyro_z_(i,j) = matrix_gyro_z_(i,j+1);
		}
	}
}

/*********************************************************************/
/*********************************************************************/

void proto_functions::loadGyro()
{
  int i;
  
  for(i=0; i<7; i++)
  { 
    matrix_gyro_x_(i,size_-1) = tmp_gyro_(i,0);
    matrix_gyro_y_(i,size_-1) = tmp_gyro_(i,1);
    matrix_gyro_z_(i,size_-1) = tmp_gyro_(i,2);
   }
}


/*********************************************************************/
/*********************************************************************/

bool proto_functions::GyroControl()
{
	int i = hit_imu_,j;
	int f = 8;
	float max_av = 500;
	float min_av = 20;
	float max_gx = 0, max_gy = 0, max_gz = 0;
	int count_gyro_x = 0, count_gyro_y = 0, count_gyro_z = 0;
	int gap = 2;


	//computing the counters for each gyroscope
	switch (hit_imu_)
	{
		case 0:	//case index
		{
			for (j=3; j<f; j++)
			{

				if (fabs(matrix_gyro_x_(i,j)) >= min_av && fabs(matrix_gyro_x_(i,j)) > fabs(max_gx) && fabs(matrix_gyro_x_(i,j)) < max_av && matrix_gyro_x_(i,j) < 0)
				{
					max_gx = matrix_gyro_x_(i,j);
					count_gyro_x ++;
				}
				
				if (fabs(matrix_gyro_y_(i,j)) >= min_av && fabs(matrix_gyro_y_(i,j)) > fabs(max_gy) && fabs(matrix_gyro_y_(i,j)) < max_av)
				{
					max_gy = matrix_gyro_y_(i,j);
					count_gyro_y ++;
				}
				
				if (fabs(matrix_gyro_z_(i,j)) >= min_av && fabs(matrix_gyro_z_(i,j)) > fabs(max_gz) && fabs(matrix_gyro_z_(i,j)) < max_av && matrix_gyro_z_(i,j) < 0)
				{
					max_gz = matrix_gyro_z_(i,j);
					count_gyro_z ++;
				}
			}
		

			
			if (fabs(max_gz) > fabs(max_gx) && count_gyro_z >= gap)	//side
			{
					axis_ = 0;
					return true;	
			}
			else if (fabs(max_gx) > (fabs(max_gz)+min_av) && count_gyro_x >= gap )
			{
					axis_ = 2;
					return true;
			}
			else if (fabs(max_gx) < (fabs(max_gz)+min_av) && count_gyro_x >= gap )
			{
					axis_ = 1;
					return true;
			}
			else
				return false;


			break;

		}

		case 1:	// case ring
		{
			for (j=3; j<f; j++)
			{

				if (fabs(matrix_gyro_x_(i,j)) >= min_av && fabs(matrix_gyro_x_(i,j)) > fabs(max_gx) && fabs(matrix_gyro_x_(i,j)) < max_av && matrix_gyro_x_(i,j) < 0)
				{
					max_gx = matrix_gyro_x_(i,j);
					count_gyro_x ++;
				}
				
				if (fabs(matrix_gyro_y_(i,j)) >= min_av && fabs(matrix_gyro_y_(i,j)) > fabs(max_gy) && fabs(matrix_gyro_y_(i,j)) < max_av)
				{
					max_gy = matrix_gyro_y_(i,j);
					count_gyro_y ++;
				}
				
				if (fabs(matrix_gyro_z_(i,j)) >= min_av && fabs(matrix_gyro_z_(i,j)) > fabs(max_gz) && fabs(matrix_gyro_z_(i,j)) < max_av)
				{
					max_gz = matrix_gyro_z_(i,j);
					count_gyro_z ++;
				}
			}

			
			if (fabs(max_gz) > fabs(max_gx) && count_gyro_z >= gap)	//side (never occurs)
			{
					axis_ = 0;
					return true;	
			}
			else if (fabs(max_gx) > (fabs(max_gz)+min_av) && count_gyro_x >= gap )
			{
					axis_ = 2;
					return true;
			}
			else if (fabs(max_gx) < (fabs(max_gz)+min_av) && count_gyro_x >= gap )
			{
					axis_ = 1;
					return true;
			}
			else
				return false;

			break;

		}

		case 2:	// case middle
		{
			for (j=3; j<f; j++)
			{
				//control on middle
				if (fabs(matrix_gyro_x_(i,j)) >= min_av && fabs(matrix_gyro_x_(i,j)) > fabs(max_gx) && fabs(matrix_gyro_x_(i,j)) < max_av && matrix_gyro_x_(i,j) < 0)
				{
					max_gx = matrix_gyro_x_(i,j);
					count_gyro_x ++;
				}
				
				if (fabs(matrix_gyro_y_(i,j)) >= min_av && fabs(matrix_gyro_y_(i,j)) > fabs(max_gy) && fabs(matrix_gyro_y_(i,j)) < max_av)
				{
					max_gy = matrix_gyro_y_(i,j);
					count_gyro_y ++;
				}
				
				if (fabs(matrix_gyro_z_(i,j)) >= min_av && fabs(matrix_gyro_z_(i,j)) > fabs(max_gz) && fabs(matrix_gyro_z_(i,j)) < max_av)
				{
					max_gz = matrix_gyro_z_(i,j);
					count_gyro_z ++;
				}
			}

				
			if (fabs(max_gz) > fabs(max_gx) && count_gyro_z >= gap)	//side (never occurs)
			{
					axis_ = 0;
					return true;	
			}
			else if (fabs(max_gx) > (fabs(max_gz)+min_av) && count_gyro_x >= gap )
			{
					axis_ = 2;
					return true;
			}
			else if (fabs(max_gx) < (fabs(max_gz)+min_av) && count_gyro_x >= gap )
			{
					axis_ = 1;
					return true;
			}
			else
				return false;

				break;

		}

		case 3:	//case index
		{
			for (j=3; j<f; j++)
			{

				if (fabs(matrix_gyro_x_(i,j)) >= min_av && fabs(matrix_gyro_x_(i,j)) > fabs(max_gx) && fabs(matrix_gyro_x_(i,j)) < max_av && matrix_gyro_x_(i,j) < 0)
				{
					max_gx = matrix_gyro_x_(i,j);
					count_gyro_x ++;
				}
				
				if (fabs(matrix_gyro_y_(i,j)) >= min_av && fabs(matrix_gyro_y_(i,j)) > fabs(max_gy) && fabs(matrix_gyro_y_(i,j)) < max_av)
				{
					max_gy = matrix_gyro_y_(i,j);
					count_gyro_y ++;
				}
				
				if (fabs(matrix_gyro_z_(i,j)) >= min_av && fabs(matrix_gyro_z_(i,j)) > fabs(max_gz) && fabs(matrix_gyro_z_(i,j)) < max_av && matrix_gyro_z_(i,j) > 0)
				{
					max_gz = matrix_gyro_z_(i,j);
					count_gyro_z ++;
				}
			}


			
			if (fabs(max_gz) > fabs(max_gx) && count_gyro_z >= gap)	//side
			{
					axis_ = 0;
					return true;	
			}
			else if (fabs(max_gx) > (fabs(max_gz)+min_av) && count_gyro_x >= gap )
			{
					axis_ = 2;
					return true;
			}
			else if (fabs(max_gx) < (fabs(max_gz)+min_av) && count_gyro_x >= gap )
			{
					axis_ = 1;
					return true;
			}
			else
				return false;

			break;

		}

		case 4:	//case thumb
		{
			for (j=3; j<f; j++)
			{

				if (fabs(matrix_gyro_x_(i,j)) >= min_av && fabs(matrix_gyro_x_(i,j)) > fabs(max_gx) && fabs(matrix_gyro_x_(i,j)) < max_av)
				{
					max_gx = matrix_gyro_x_(i,j);
					count_gyro_x ++;
				}
				
				if (fabs(matrix_gyro_y_(i,j)) >= min_av && fabs(matrix_gyro_y_(i,j)) > fabs(max_gy) && fabs(matrix_gyro_y_(i,j)) < max_av)
				{
					max_gy = matrix_gyro_y_(i,j);
					count_gyro_y ++;
				}
				
				if (fabs(matrix_gyro_z_(i,j)) >= min_av && fabs(matrix_gyro_z_(i,j)) > fabs(max_gz) && fabs(matrix_gyro_z_(i,j)) < max_av && matrix_gyro_z_(i,j) > 0)
				{
					max_gz = matrix_gyro_z_(i,j);
					count_gyro_z ++;
				}
			}

			if (fabs(max_gx) > fabs(max_gz))
			{
				if (count_gyro_x >= gap && max_gx > 0)	// vertical
				{
					axis_ = 2;
					return true;
				}
				else if (count_gyro_x >= gap && max_gx < 0)	// frontal
				{
					axis_ = 1;
					return true;
				}
			}
			else if (fabs(max_gz) > fabs(max_gx))	//side
			{
				if(count_gyro_z >= gap)
				{
					axis_ = 0;
					return true;
				}
			}
			else
				return false;

			break;

		}

		default:
				break;
	}

}


/*********************************************************************/
/*********************************************************************/


/*###################################################################*/
/*#####################<<<<<COMMON PART>>>>>#########################*/
/*###################################################################*/


/*********************************************************************/
/*********************************************************************/

void proto_functions::silence()
{

	if (wait_flag_)
	{
		silence_++;
		if (silence_ == 50)
		{
			wait_flag_ = false;
			// a_acquisition_flag_ = true;
			// g_acquisition_flag_ = true;
			silence_ = 0;
			std::cout <<"\n\nSilence End\n\n";
		}
	}
}

/*********************************************************************/
/*********************************************************************/

void proto_functions::protoManager()
{
	if (!a_acquisition_flag_ && !g_acquisition_flag_)
	waitCall();

	if (work_flag_ && !a_acquisition_flag_ && !g_acquisition_flag_ )
	{
		/* shift */	
		updateAccMatrix();

		updateGyroMatrix();

		/* filerting data */
		filterData();

		/* load new data */
		loadGyro();

		loadAcc();

		/* control */
		if (!wait_flag_)
		{
			if (AccControl())
					if (GyroControl())
						possibleFinger();
		}

		work_flag_ = false;
		a_acquisition_flag_ = true;
		g_acquisition_flag_ = true;

	silence();
	}

}