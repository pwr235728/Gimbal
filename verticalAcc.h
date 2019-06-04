#ifndef VERTICALACC_H_
#define VERTICALACC_H_

#include <math.h>
#include <stdint.h>


// Wartość przyśpieszenia ziemskiego we Wrocławiu
#define G 9.8113


#define PI 3.14159265359
#define deg2rad(x) (PI*x/180)

typedef struct{
	float data[3];
}vector_t;

typedef struct{
	float data[9];
}matrix_t;

matrix_t matrix_rot_pitch( float angle);
matrix_t matrix_rot_roll( float angle);

vector_t vector_create(float x, float y, float z);
static float vector_multiply(float *col, float *row);
vector_t matrix_vector_multiply(matrix_t m, vector_t v);
matrix_t transpose(matrix_t m);
matrix_t multiply(matrix_t m1, matrix_t m2);

vector_t get_acc(vector_t imu_acc, vector_t imu_gyro);

#endif /* VERTICALACC_H_ */
