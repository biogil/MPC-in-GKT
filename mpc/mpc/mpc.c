#include "mpc.h"
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

int main()
{
	mpc_go();

	return 0;
}

void mpc_go()
{
	mpc m_mpc;
	Input u;
	Output y;
	RealModel m_rm;

	PID gkt_valve3_pid;
	PID gkt_valve5_pid;
	PID gkt_valve6_pid;
	PID gkt_heating_pid;

	FILE* fp;
	errno_t err = fopen_s(&fp, "output_mpc.txt", "w");
	if (fp == NULL) {
		printf("Error! Could not open file\n");
	}

	u.gkt_valve1_value = 0.5;
	u.gkt_valve2_value = 0.5;
	u.gkt_valve3_value = 0.5;
	u.gkt_valve4_value = 0.5;
	u.gkt_valve5_value = 0.5;
	u.gkt_valve6_value = 0.5;
	u.gkt_heating_value = 10000;

	int  i, n = 100;

	gkt_init(&m_rm);
	gkt_valve3_control_init(&gkt_valve3_pid);
	gkt_valve5_control_init(&gkt_valve5_pid);
	gkt_valve6_control_init(&gkt_valve6_pid);
	gkt_heating_control_init(&gkt_heating_pid);

	y = gkt_go(&m_rm, u);

	InitMPC(&m_mpc, u, y, m_rm);

	for (i = 0; i < 1000; i++){
		u = Optimization(&m_mpc);
		y = gkt_go(&m_rm, u);
		u.gkt_valve3_value = gkt_valve3_control_go(&gkt_valve3_pid, u.gkt_valve3_value, m_rm.gkt_cabin_colding.P, 500000);
		u.gkt_valve5_value = gkt_valve5_control_go(&gkt_valve5_pid, u.gkt_valve5_value, m_rm.gkt_cabin_back.P, 20000);
		u.gkt_valve6_value = gkt_valve6_control_go(&gkt_valve6_pid, u.gkt_valve6_value, m_rm.gkt_cabin_back.P, 20000);
		u.gkt_heating_value = gkt_heating_control_go(&gkt_heating_pid, u.gkt_heating_value, m_rm.gkt_heating.Tout, 350);
		UpdateMPC(&m_mpc, u, y, m_rm);
		printf("ConT: %.6f ConP: %.6f P: %.6f\n", y.ConT, y.ConP, y.P);
		printf("u1: %.6f u2: %.6f u4: %.6f\n", u.gkt_valve1_value, u.gkt_valve2_value, u.gkt_valve4_value);
		printf("u3: %.6f u5: %.6f u6: %.6f\n", u.gkt_valve3_value, u.gkt_valve5_value, u.gkt_valve6_value);
		printf("bestfitness:%.2f\n\n", m_mpc.s.GFitness);
		printf("%d\n", i);
	}
	for (i = 0; i < 10; i++) {
		u = Optimization(&m_mpc);
		y = gkt_go(&m_rm, u);
		u.gkt_valve3_value = gkt_valve3_control_go(&gkt_valve3_pid, u.gkt_valve3_value, m_rm.gkt_cabin_colding.P, 500000);
		u.gkt_valve5_value = gkt_valve5_control_go(&gkt_valve5_pid, u.gkt_valve5_value, m_rm.gkt_cabin_back.P, 20000);
		u.gkt_valve6_value = gkt_valve6_control_go(&gkt_valve6_pid, u.gkt_valve6_value, m_rm.gkt_cabin_back.P, 20000);
		u.gkt_heating_value = gkt_heating_control_go(&gkt_heating_pid, u.gkt_heating_value, m_rm.gkt_heating.Tout, 350);
		UpdateMPC(&m_mpc, u, y, m_rm);
		printf("ConT: %.6f ConP: %.6f P: %.6f\n", y.ConT, y.ConP, y.P);
		fprintf(fp, "%.6f %.6f %.6f\n", y.ConT, y.ConP, y.P);
		printf("u1: %.6f u2: %.6f u4: %.6f\n", u.gkt_valve1_value, u.gkt_valve2_value, u.gkt_valve4_value);
		printf("u3: %.6f u5: %.6f u6: %.6f\n", u.gkt_valve3_value, u.gkt_valve5_value, u.gkt_valve6_value);
		printf("bestfitness:%.2f\n\n", m_mpc.s.GFitness);
		printf("%d\n", i);
	}
	m_rm.gkt_engine.Acr = 1;
	printf("\n********************Acr Change: Acr = 1***********************\n\n");
	for (i = 0; i < 100; i++) {
		u = Optimization(&m_mpc);
		y = gkt_go(&m_rm, u);
		u.gkt_valve3_value = gkt_valve3_control_go(&gkt_valve3_pid, u.gkt_valve3_value, m_rm.gkt_cabin_colding.P, 500000);
		u.gkt_valve5_value = gkt_valve5_control_go(&gkt_valve5_pid, u.gkt_valve5_value, m_rm.gkt_cabin_back.P, 20000);
		u.gkt_valve6_value = gkt_valve6_control_go(&gkt_valve6_pid, u.gkt_valve6_value, m_rm.gkt_cabin_back.P, 20000);
		u.gkt_heating_value = gkt_heating_control_go(&gkt_heating_pid, u.gkt_heating_value, m_rm.gkt_heating.Tout, 350);
		UpdateMPC(&m_mpc, u, y, m_rm);
		printf("ConT: %.6f ConP: %.6f P: %.6f\n", y.ConT, y.ConP, y.P);
		fprintf(fp, "%.6f %.6f %.6f\n", y.ConT, y.ConP, y.P);
		printf("u1: %.6f u2: %.6f u4: %.6f\n", u.gkt_valve1_value, u.gkt_valve2_value, u.gkt_valve4_value);
		printf("u3: %.6f u5: %.6f u6: %.6f\n", u.gkt_valve3_value, u.gkt_valve5_value, u.gkt_valve6_value);
		printf("bestfitness:%.2f\n\n", m_mpc.s.GFitness);
		printf("%d\n", i);
	}
	fclose(fp);
	getchar();
}

void InitSwarm(swarm *s){
	int i, j, k;

	s->WMax = 1.4;
	s->WMin = 0.4;
	s->C1 = 2.0;
	s->C2 = 2.0;
	for (i = 0; i < XDim; i++){
		for (j = 0; j < CDim; j++){
			if (j == 0){
				s->Xup[i][j] = 1;
				s->Xdown[i][j] = -1;
				s->Vmax[i][j] = 0.002;
			}
			if (j == 1){
				s->Xup[i][j] = 100;
				s->Xdown[i][j] = 1;
				s->Vmax[i][j] = 0.2;
			}
		}
	}

	srand((unsigned)time(NULL));
	for (i = 0; i < PNum; i++){
		for (j = 0; j < XDim; j++){
			for (k = 0; k < CDim; k++){
				s->P[i].X[j][k] = rand() / (double)RAND_MAX*(s->Xup[j][k] - s->Xdown[j][k]) + s->Xdown[j][k];
				s->P[i].V[j][k] = rand() / (double)RAND_MAX* s->Vmax[j][k] * 2 - s->Vmax[j][k];
			}
		}
	}
}

void UpdateMPC(mpc *my_mpc, Input u, Output y, RealModel m_rm){
	my_mpc->u = u;
	my_mpc->y = y;
	my_mpc->m_rm = m_rm;
	InitSwarm(&(my_mpc->s));
	return ;
}

void InitMPC(mpc *my_mpc, Input u, Output y, RealModel m_rm){
	my_mpc->StepTime = 0.01;
	my_mpc->Object.ConP = 200000;
	my_mpc->Object.ConT = 250;
	UpdateMPC(my_mpc, u, y, m_rm);
}

void UpdateMPCInObj(mpc *my_mpc, Output Object){
	my_mpc->Object = Object;
}

double GetU(double a, double b, double i){

	double y;

	y = a*((1-exp(-2/b*(i+1)))/(1+exp(-2/b*(i+1))));

	return y;
}

void PredictModel(RealModel m_rm, pmio *m_pmio){

	int i;

	RealModel m_pm;
	PID gkt_valve3_pid;
	PID gkt_valve5_pid;
	PID gkt_valve6_pid;
	PID gkt_heating_pid;

	Input u[PredictStep];
	Output y[PredictStep];

	m_pm = m_rm;

	gkt_valve3_control_init(&gkt_valve3_pid);
	gkt_valve5_control_init(&gkt_valve5_pid);
	gkt_valve6_control_init(&gkt_valve6_pid);
	gkt_heating_control_init(&gkt_heating_pid);

	for (i = 0; i < PredictStep; i++){
		if (i < ControlStep)
			u[i] = m_pmio->u[i];
		else
			u[i] = m_pmio->u[ControlStep - 1];
	}

	u[0].gkt_valve3_value = m_pmio->u[0].gkt_valve3_value;
	u[0].gkt_valve5_value = m_pmio->u[0].gkt_valve5_value;
	u[0].gkt_valve6_value = m_pmio->u[0].gkt_valve6_value;
	u[0].gkt_heating_value = m_pmio->u[0].gkt_heating_value;

	for (i = 0; i < PredictStep; i++){
		if (i > 0)
		{
			u[i].gkt_valve3_value = gkt_valve3_control_go(&gkt_valve3_pid, u[i-1].gkt_valve3_value, m_pm.gkt_cabin_colding.P, 500000);
			u[i].gkt_valve5_value = gkt_valve5_control_go(&gkt_valve5_pid, u[i-1].gkt_valve5_value, m_pm.gkt_cabin_back.P, 20000);
			u[i].gkt_valve6_value = gkt_valve6_control_go(&gkt_valve6_pid, u[i-1].gkt_valve6_value, m_pm.gkt_cabin_back.P, 20000);
			u[i].gkt_heating_value = gkt_heating_control_go(&gkt_heating_pid, u[i-1].gkt_heating_value, m_pm.gkt_heating.Tout, 350);
		}
		y[i] = gkt_go(&m_pm, u[i]);
	}

	for (i = 0; i < PredictStep; i++){
		m_pmio->u[i] = u[i];
		m_pmio->y[i] = y[i];
	}
}

double ComputeOptimumValue(pmio m_pmio, Output object){

	int i;
	double J;
	double R;
	double x1, x2;

	R = 1;

	J = 0;

	for (i = 0; i < PredictStep; i++){
		x1 = (m_pmio.y[i].ConT - object.ConT)*(m_pmio.y[i].ConT - object.ConT) / (object.ConT * object.ConT);
		x2 = (m_pmio.y[i].ConP - object.ConP)*(m_pmio.y[i].ConP - object.ConP) / (object.ConP * object.ConP);

		J += R*(10*x1+x2);
		R *= 0.1;
	}

	return J;
}

double Fitness(double X[][2], RealModel m_rm, Input u, Output Object){

	int i;
	double J;

	pmio m_pmio;

	for (i = 0; i < ControlStep; i++){
		m_pmio.u[i].gkt_valve1_value = u.gkt_valve1_value + GetU(X[0][0], X[0][1], i);
		m_pmio.u[i].gkt_valve2_value = u.gkt_valve2_value + GetU(X[1][0], X[1][1], i);
		m_pmio.u[i].gkt_valve4_value = u.gkt_valve4_value + GetU(X[2][0], X[2][1], i);
		if (m_pmio.u[i].gkt_valve1_value > 1)
			m_pmio.u[i].gkt_valve1_value = 1;
		if (m_pmio.u[i].gkt_valve1_value < 0)
			m_pmio.u[i].gkt_valve1_value = 0;
		if (m_pmio.u[i].gkt_valve2_value > 1)
			m_pmio.u[i].gkt_valve2_value = 1;
		if (m_pmio.u[i].gkt_valve2_value < 0)
			m_pmio.u[i].gkt_valve2_value = 0;
		if (m_pmio.u[i].gkt_valve4_value > 1)
			m_pmio.u[i].gkt_valve4_value = 1;
		if (m_pmio.u[i].gkt_valve4_value < 0)
			m_pmio.u[i].gkt_valve4_value = 0;
		m_pmio.u[0].gkt_valve3_value = u.gkt_valve3_value;
		m_pmio.u[0].gkt_valve5_value = u.gkt_valve5_value;
		m_pmio.u[0].gkt_valve6_value = u.gkt_valve6_value;
		m_pmio.u[0].gkt_heating_value = u.gkt_heating_value;
	}
	PredictModel(m_rm, &m_pmio);
	J = ComputeOptimumValue(m_pmio, Object);
	return J;
}

void ComputFitness(swarm *s, RealModel m_rm, Input u, Output Object){

	int i;

	for (i = 0; i < PNum; i++){
		s->P[i].Fitness = Fitness(s->P[i].X, m_rm, u, Object);
	}
}

void UpdateVandX(swarm *s){

	int i, j, k;

	srand((unsigned)time(NULL));

	for (i = 0; i < PNum; i++){
		for (j = 0; j < XDim; j++){
			for (k = 0; k < CDim; k++){
				s->P[i].V[j][k] = s->W * s->P[i].V[j][k] +
					rand() / (double)RAND_MAX * s->C1 * (s->P[i].PX[j][k] - s->P[i].X[j][k]) +
					rand() / (double)RAND_MAX * s->C2 * (s->GBest[j][k] - s->P[i].X[j][k]);
			}
		}
		
		for (j = 0; j < XDim; j++){
			for (k = 0; k<CDim; k++){
				if (s->P[i].V[j][k] > s->Vmax[j][k])
					s->P[i].V[j][k] = s->Vmax[j][k];
				if (s->P[i].V[j][k] < -s->Vmax[j][k])
					s->P[i].V[j][k] = -s->Vmax[j][k];
			}
		}
		for (j = 0; j < XDim; j++){
			for (k = 0; k<CDim; k++){
				s->P[i].X[j][k] += s->P[i].V[j][k];
				if (s->P[i].X[j][k] > s->Xup[j][k])
					s->P[i].X[j][k] = s->Xup[j][k];
				if (s->P[i].X[j][k] < s->Xdown[j][k])
					s->P[i].X[j][k] = s->Xdown[j][k];
			}
		}
	}
}

void UpdatePandGbest(swarm *s, int FirstFlag, RealModel m_rm, Input u, Output Object){
	int i, j, k;

	if (FirstFlag == 1){
		for (i = 0; i < PNum; i++){
			for (j = 0; j < XDim; j++){
				for (k = 0; k < CDim; k++){
					s->P[i].PX[j][k] = s->P[i].X[j][k];
					s->P[i].PFitness = s->P[i].Fitness;
				}
			}
		}
		for (j = 0; j < XDim; j++){
			for (k = 0; k < CDim; k++){
				s->GBest[j][k] = s->P[0].PX[j][k];
				s->GFitness = s->P[0].PFitness;
			}
		}
	}
	if (FirstFlag == 0){
		for (i = 0; i < PNum; i++){
			if (s->P[i].Fitness > Fitness(s->P[i].PX, m_rm, u, Object)){
				for (j = 0; j < XDim; j++){
					for (k = 0; k < CDim; k++){
						s->P[i].PX[j][k] = s->P[i].X[j][k];
						s->P[i].PFitness = s->P[i].Fitness;
					}
				}
			}
		}
		for (i = 0; i < PNum; i++){
			if (s->GFitness > Fitness(s->P[i].PX, m_rm, u, Object)){
				for (j = 0; j < XDim; j++){
					for (k = 0; k < CDim; k++){
						s->GBest[j][k] = s->P[i].PX[j][k];
						s->GFitness = s->P[i].PFitness;
					}
				}
			}
		}
	}
	
}

void GetInput(double X[][2], Input *m_input, Input u){
	m_input->gkt_valve1_value = u.gkt_valve1_value + GetU(X[0][0], X[0][1], 0);
	m_input->gkt_valve2_value = u.gkt_valve2_value + GetU(X[1][0], X[1][1], 0);
	m_input->gkt_valve4_value = u.gkt_valve4_value + GetU(X[2][0], X[2][1], 0);
	if (m_input->gkt_valve1_value > 1)
		m_input->gkt_valve1_value = 1;
	if (m_input->gkt_valve1_value < 0)
		m_input->gkt_valve1_value = 0;
	if (m_input->gkt_valve2_value > 1)
		m_input->gkt_valve2_value = 1;
	if (m_input->gkt_valve2_value < 0)
		m_input->gkt_valve2_value = 0;
	if (m_input->gkt_valve4_value > 1)
		m_input->gkt_valve4_value = 1;
	if (m_input->gkt_valve4_value < 0)
		m_input->gkt_valve4_value = 0;
	m_input->gkt_valve3_value = u.gkt_valve3_value;
	m_input->gkt_valve5_value = u.gkt_valve5_value;
	m_input->gkt_valve6_value = u.gkt_valve6_value;
	m_input->gkt_heating_value = u.gkt_heating_value;
}

Input Optimization(mpc *my_mpc){

	int i, iters_num;

	Input BestInput;

	iters_num = 100; //µü´ú´ÎÊý£»

	ComputFitness(&(my_mpc->s), my_mpc->m_rm, my_mpc->u, my_mpc->Object);
	UpdatePandGbest(&(my_mpc->s), 1, my_mpc->m_rm, my_mpc->u, my_mpc->Object);

	for(i=0;i<iters_num;i++){
		my_mpc->s.W = my_mpc->s.WMax - i *(my_mpc->s.WMax - my_mpc->s.WMin) / iters_num;
		UpdateVandX(&(my_mpc->s));
		ComputFitness(&(my_mpc->s), my_mpc->m_rm, my_mpc->u, my_mpc->Object);
		UpdatePandGbest(&(my_mpc->s), 0, my_mpc->m_rm, my_mpc->u, my_mpc->Object);
	}
	GetInput(my_mpc->s.GBest, &BestInput, my_mpc->u);

	return BestInput;
}


