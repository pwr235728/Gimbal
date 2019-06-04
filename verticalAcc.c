#include "verticalAcc.h"

vector_t vector_create(float x, float y, float z)
{
	vector_t v;
	v.data[0] = x;
	v.data[1] = y;
	v.data[2] = z;

	return v;
}

matrix_t matrix_rot_pitch( float angle)
{
	matrix_t m;
	m.data[0] = cos(angle);
	m.data[1] = 0;
	m.data[2] = sin(angle);
	m.data[3] = 0;
	m.data[4] = 1;
	m.data[5] = 0;
	m.data[6] = -sin(angle);
	m.data[7] = 0;
	m.data[8] = cos(angle);

	return m;
}
matrix_t matrix_rot_roll( float angle)
{
	matrix_t m;
	m.data[0] = 1;
	m.data[1] = 0;
	m.data[2] = 0;
	m.data[3] = 0;
	m.data[4] = cos(angle);
	m.data[5] = -sin(angle);
	m.data[6] = 0;
	m.data[7] = sin(angle);
	m.data[8] = cos(angle);

	return m;
}

static float vector_multiply(float *col, float *row){
	float sum;
	unsigned int i;
	sum = 0;
	for(i = 0; i < 3; i++){
		sum += col[i] * row[i];
	}
	return sum;
}


vector_t matrix_vector_multiply(matrix_t m, vector_t v)
{
	vector_t out;
	for(int i=0;i<3;i++){
		out.data[i] = 0;
		for(int j=0;j<3;j++){
			out.data[i] += m.data[i*3 + j]*v.data[j];
		}
	}
	return out;
}

matrix_t transpose(matrix_t m){
	matrix_t trans;
	unsigned int i, j;

	for(i = 0; i < 3; i++){
		for(j = 0; j < 3; j++)
			trans.data[i*3+j] = m.data[j*3+i];
	}
	return trans;
}

matrix_t multiply(matrix_t m1, matrix_t m2){
	matrix_t product, trans;
	unsigned int i, j;

	trans = transpose(m2);

	for(i = 0; i < 3; i++){
		for(j = 0; j < 3; j++){
			product.data[i*3+j] = vector_multiply(&m1.data[i*3], &trans.data[j*3]);
		}
	}
	return product;
}

vector_t get_acc(vector_t imu_acc, vector_t imu_gyro)
{
	matrix_t rot_pitch;
	matrix_t rot_roll;
	matrix_t rot;

	vector_t acc_in_world;

	rot_pitch = matrix_rot_pitch(deg2rad(imu_gyro.data[1])); // 0- roll, 1- pitch, 2- yaw
	rot_roll = matrix_rot_roll(deg2rad(imu_gyro.data[0]));

	rot = multiply(rot_pitch, rot_roll);

	acc_in_world = matrix_vector_multiply(rot, imu_acc);
	acc_in_world.data[2] -= G;

	return acc_in_world;
}

