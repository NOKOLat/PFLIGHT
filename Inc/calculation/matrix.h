/*
 * mat_cal.h
 *
 *  Created on: Feb 20, 2025
 *      Author: takut
 */

#ifndef INC_MAT_CAL_H_
#define INC_MAT_CAL_H_

//#include "arm_math.h" //計算ライブラリdep

void matcal(arm_matrix_instance_f32 A,arm_matrix_instance_f32 B,arm_matrix_instance_f32 C,char c){
	switch(c){
		case '+': arm_mat_add_f32(&A,&B,&C); break;
		case '-': arm_mat_sub_f32(&A,&B,&C); break;
		case '*': arm_mat_mult_f32(&A,&B,&C); break;
		case 't': arm_mat_trans_f32(&A,&C); break;
		case 'i': arm_mat_inverse_f32(&A,&C); break;
		default : break;
	}
}

//----------4行行列----------//

//4行1列 4行1列
void mat41(float x[4],float y[4],float answer[4],char c){
	arm_matrix_instance_f32 A ={4,1,(float32_t *)x};
	arm_matrix_instance_f32 B ={4,1,(float32_t *)y};
	arm_matrix_instance_f32 C ={4,1,(float32_t *)answer};
	matcal(A,B,C,c);
}

//4行4列 4行1列
void mat441(float x[4][4],float y[4],float answer[4],char c){
	arm_matrix_instance_f32 A ={4,4,(float32_t *)x};
	arm_matrix_instance_f32 B ={4,1,(float32_t *)y};
	arm_matrix_instance_f32 C ={4,1,(float32_t *)answer};
	matcal(A,B,C,c);
}

//4行4列 4行4列
void mat44(float x[4][4],float y[4][4],float answer[4][4],char c){
	arm_matrix_instance_f32 A ={4,4,(float32_t *)x};
	arm_matrix_instance_f32 B ={4,4,(float32_t *)y};
	arm_matrix_instance_f32 C ={4,4,(float32_t *)answer};
	matcal(A,B,C,c);
}

//4行4列*4行4列*4行4列
void mat444(float x[4][4],float y[4][4],float z[4][4],float answer[4][4]){
	arm_matrix_instance_f32 A ={4,4,(float32_t *)x};
	arm_matrix_instance_f32 B ={4,4,(float32_t *)y};
	arm_matrix_instance_f32 C ={4,4,(float32_t *)z};
	arm_matrix_instance_f32 D ={4,4,(float32_t *)answer};
	double buf[4][4];
	arm_matrix_instance_f32 E ={4,4,(float32_t *)buf};
	arm_mat_mult_f32(&A,&B,&E);
	arm_mat_mult_f32(&E,&C,&D);
}


//----------3行行列----------//

//3行1列 3行1列
void mat31(float x[3],float y[3],float answer[3],char c){
	arm_matrix_instance_f32 A ={3,1,(float32_t *)x};
	arm_matrix_instance_f32 B ={3,1,(float32_t *)y};
	arm_matrix_instance_f32 C ={3,1,(float32_t *)answer};
	matcal(A,B,C,c);
}

//3行3列 3行1列
void mat331(float x[3][3],float y[3],float answer[3],char c){
	arm_matrix_instance_f32 A ={3,3,(float32_t *)x};
	arm_matrix_instance_f32 B ={3,1,(float32_t *)y};
	arm_matrix_instance_f32 C ={3,1,(float32_t *)answer};
	matcal(A,B,C,c);
}

//3行3列 3行3列
void mat33(float x[3][3],float y[3][3],float answer[3][3],char c){
	arm_matrix_instance_f32 A ={3,3,(float32_t *)x};
	arm_matrix_instance_f32 B ={3,3,(float32_t *)y};
	arm_matrix_instance_f32 C ={3,3,(float32_t *)answer};
	matcal(A,B,C,c);
}

//3行3列*3行3列*3行3列
void mat333(float x[3][3],float y[3][3],float z[3][3],float answer[3][3]){
	arm_matrix_instance_f32 A ={3,3,(float32_t *)x};
	arm_matrix_instance_f32 B ={3,3,(float32_t *)y};
	arm_matrix_instance_f32 C ={3,3,(float32_t *)z};
	arm_matrix_instance_f32 D ={3,3,(float32_t *)answer};
	double buf[3][3];
	arm_matrix_instance_f32 E ={3,3,(float32_t *)buf};
	arm_mat_mult_f32(&A,&B,&E);
	arm_mat_mult_f32(&E,&C,&D);
}


#endif /* INC_MAT_CAL_H_ */
