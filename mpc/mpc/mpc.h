#ifndef _MPC_H_
#define _MPC_H_

#include "gkt.h"

//����ʱ��
#define PredictStep 20 //Ԥ�ⲽ����
#define ControlStep 10 //���Ʋ�����
#define XDim 3 //���������
#define YDim 2 //���������
#define CDim 2 //���������
#define PNum 20 //��Ⱥ��������

typedef struct gktin Input;
typedef struct gktout Output;
typedef struct gkt RealModel;
typedef struct pid PID;

typedef struct PARTICLE{
	double X[XDim][CDim];
	double V[XDim][CDim];
	double PX[XDim][CDim];
	double Fitness;
	double PFitness;
}particle;

typedef struct SWARM{
	particle P[PNum];
	int GBestIndex;
	double GBest[XDim][CDim];
	double GFitness;
	double W;
	double WMax;
	double WMin;
	double C1;
	double C2;
	double Xup[XDim][CDim];
	double Xdown[XDim][CDim];
	double Vmax[XDim][CDim];
}swarm;

typedef struct MPC{
	double StepTime; //����ʱ�䣻
	swarm s;
	Output Object;
	RealModel m_rm;

	//�������������
	/*****�����ﶨ������*****/
	//���� Input u;
	Input u;
	/************************/

	/*****�����ﶨ�����*****/
	//���� Output y;
	Output y;
	/************************/

}mpc;

typedef struct PredictModelIO{
	Input u[PredictStep];
	Output y[PredictStep];
}pmio;

void InitSwarm(swarm *s);
double mohu(double x_up, double x_down, double y_up, double y_down, double x);
void UpdateMPC(mpc *my_mpc, Input u, Output y, RealModel m_rm);
void InitMPC(mpc *my_mpc, Input u, Output y, RealModel m_rm);
double GetU(double a, double b, double i);
void PredictModel(RealModel m_rm, pmio* m_pmio);
double ComputeOptimumValue(pmio m_pmio, Output object);
double Fitness(double X[][2], RealModel m_rm, Input u, Output Object);
void ComputFitness(swarm *s, RealModel m_rm, Input u, Output Object);
void UpdateVandX(swarm *s);
void UpdatePandGbest(swarm *s, int FirstFlag, RealModel m_rm, Input u, Output Object);
void GetInput(double X[][2], Input *m_input, Input u);
Input Optimization(mpc *m_mpc);
void mpc_go();

#endif