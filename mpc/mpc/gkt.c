#include "gkt.h"
#include<stdio.h>
#include<math.h>

#define PI 3.14159265

double C_LineChazhi(double *x, double *y, int n, double nx)
{
	int i;

	if (n<2)return y[0];


	for (i = 0; i<n; i++)
	{
		if (x[i]>nx)break;
	}
	if (i == 0)i = 1;
	if (i == n)i = n - 1;
	if (x[i] - x[i - 1] == 0)
	{
		return y[i];
	}
	nx = (nx - x[i - 1]) / (x[i] - x[i - 1])*(y[i] - y[i - 1]) + y[i - 1];

	return nx;
}

double phi_go(double valve_u, double pin, double pout)
{
	if (valve_u < 15)
		return 0;
	else if (valve_u>90)
		valve_u = 90;
	if (pin < pout || pin == pout)
		return 0;

	double valve_ua[16];
	double phi[16];
	double phi_u;

	valve_ua[0] = 15;

	for (int i = 1; i < 16; i++)
		valve_ua[i] = valve_ua[i - 1] + 5;

	phi[0] = 0.1359; phi[1] = 0.2419; phi[2] = 0.3183; phi[3] = 0.3328;
	phi[4] = 0.3662; phi[5] = 0.3756; phi[6] = 0.3718; phi[7] = 0.3808;
	phi[8] = 0.3824; phi[9] = 0.3926; phi[10] = 0.4029; phi[11] = 0.4028;
	phi[12] = 0.3856; phi[13] = 0.3645; phi[14] = 0.3385; phi[15] = 0.3113;

	phi_u = C_LineChazhi(valve_ua, phi, 16, valve_u);

	return phi_u;
}

void valve_init(struct valve *my_valve)
{
	my_valve->d = 0.3;
	my_valve->phi = 0;
	my_valve->pin = 0;
	my_valve->pout = 0;
	my_valve->T = 0;
	my_valve->valve_u = 0;
}
double valve_go(struct valve *my_valve, double steptime)
{
	double Wa;
	double A0;
	double R;
	double valve_u_du;

	if (my_valve->valve_u > 1)
		my_valve->valve_u = 1;
	if (my_valve->valve_u < 0)
		my_valve->valve_u = 0;

	valve_u_du = my_valve->valve_u * 90;

	R = 287;

	A0 = PI*my_valve->d*my_valve->d / 4 * (1 - cos(valve_u_du*PI / 180));
	my_valve->phi = phi_go(valve_u_du, my_valve->pin, my_valve->pout);
	Wa = my_valve->phi*my_valve->pin*A0*sqrt(2 / (R * my_valve->T));

	my_valve->Wa = Wa;

	return Wa;
}

void cabin_init(struct cabin *my_cabin)
{
	my_cabin->wa_in1 = 0;
	my_cabin->wa_in2 = 0;
	my_cabin->wa_out = 0;
	my_cabin->Tin1 = 280;
	my_cabin->Tin2 = 280;
	my_cabin->T = 280;
	my_cabin->P = 100000;
	my_cabin->Cin1 = 0.01;
	my_cabin->Cin2 = 0.01;
	my_cabin->Cout = 0.01;
	my_cabin->V = 10;
}

double cabin_go(struct cabin *my_cabin, double steptime)
{
	double R = 287;
	double cp = 1000;
	double dT, dP;

	double h;
	double hin1;
	double hin2;

	{
		if (my_cabin->wa_in1 > 100000)
			my_cabin->wa_in1 = 100000;
		if (my_cabin->wa_in2 > 100000)
			my_cabin->wa_in2 = 100000;
		if (my_cabin->wa_out > 100000)
			my_cabin->wa_out = 100000;
		if (my_cabin->wa_in1 < 0)
			my_cabin->wa_in1 = 0;
		if (my_cabin->wa_in2 < 0)
			my_cabin->wa_in2 = 0;
		if (my_cabin->wa_out < 0)
			my_cabin->wa_out = 0;

		if (my_cabin->T > 100000)
			my_cabin->T = 100000;
		if (my_cabin->Tin1 > 100000)
			my_cabin->Tin1 = 100000;
		if (my_cabin->Tin2 > 100000)
			my_cabin->Tin2 = 100000;
		if (my_cabin->T < 0)
			my_cabin->T = 0;
		if (my_cabin->Tin1 < 0)
			my_cabin->Tin1 = 0;
		if (my_cabin->Tin2 < 0)
			my_cabin->Tin2 = 0;
	}

	h = cp*my_cabin->T;
	hin1 = cp*my_cabin->Tin1;
	hin2 = cp*my_cabin->Tin2;

	dT = (R*my_cabin->T / (my_cabin->P*my_cabin->V*(cp - R))*(-(my_cabin->wa_in1 + my_cabin->wa_in2 - my_cabin->wa_out))*(h - R*my_cabin->T)+
		R*my_cabin->T / (my_cabin->P*my_cabin->V*(cp - R))*((hin1 + my_cabin->Cin1*my_cabin->Cin1 / 2)*my_cabin->wa_in1 + (hin2 + my_cabin->Cin2*my_cabin->Cin2 / 2)*my_cabin->wa_in2 - (h + my_cabin->Cout*my_cabin->Cout / 2)*my_cabin->wa_out))*steptime;

	dP = ((R / my_cabin->V)*((my_cabin->T - (h - R*my_cabin->T) / (cp - R))*(my_cabin->wa_in1 + my_cabin->wa_in2 - my_cabin->wa_out)) + R / (my_cabin->V*(cp - R))*((hin1 + my_cabin->Cin1*my_cabin->Cin1 / 2)*my_cabin->wa_in1 + (hin2 + my_cabin->Cin2*my_cabin->Cin2 / 2)*my_cabin->wa_in2 - (h + my_cabin->Cout*my_cabin->Cout / 2)*my_cabin->wa_out))*steptime;

	my_cabin->T += dT;
	my_cabin->P += dP;

	return my_cabin->P;
}

void heating_init(struct heating* my_heating)
{
	my_heating->wa = 0;
	my_heating->c = 1;
	my_heating->C = 10;
	my_heating->Tin = 273.15;
	my_heating->Tout = 273.15;
	my_heating->q = 0;
}

double heating_go(struct heating* my_heating, double steptime)
{
	double dT;

	dT = (my_heating->q - my_heating->wa * my_heating->c * (my_heating->Tout - my_heating->Tin)) / my_heating->C * steptime;

	my_heating->Tout = my_heating->Tin + dT;

	return my_heating->Tout;
}

void colding_init(struct colding* my_colding)
{
	my_colding->eta = 1;
	my_colding->Pin = 100000;
	my_colding->Pout = 100000;
	my_colding->Tin = 273.15;
	my_colding->Tout = 273.15;
}

double colding_go(struct colding* my_colding)
{
	my_colding->Tout = my_colding->Tin*(1- my_colding->eta*(1-pow(my_colding->Pin / my_colding->Pout, -0.286)));

	return my_colding->Tout;
}

void engine_init(struct engine* my_engine)
{
	my_engine->Acr = 0.5;
	my_engine->ConT = 300;
	my_engine->ConP = 200000;
	my_engine->P = 90000;
	my_engine->Wa = 0;
}

double engine_go(struct engine* my_engine)
{
	double k, R;
	double K, lambda, q;

	k = 1.4;
	R = 287;

	K = sqrt(k/R*pow(2/(k+1), (k+1)/(k-1)));
	lambda = sqrt((k+1)/(k-1)*(1-pow(my_engine->P/ my_engine->ConP, (k-1)/k)));
	q = lambda * pow((k+1)/2-(k-1)/2*lambda*lambda,1/(k-1));
	my_engine->Wa = K * my_engine->ConP / sqrt(my_engine->ConT) * my_engine->Acr * q;

	return my_engine->Wa;
}

void gkt_init(struct gkt *my_gkt)
{
	my_gkt->steptime = 0.01;
	my_gkt->gkt_gktout.ConP = 100000;
	my_gkt->gkt_gktout.ConT = 280;
	my_gkt->gkt_gktout.P = 100000;
	my_gkt->Pin = 1000000;
	my_gkt->Pout = 100000;
	my_gkt->Tin = 300;
	valve_init(&(my_gkt->gkt_valve1));
	valve_init(&(my_gkt->gkt_valve2));
	valve_init(&(my_gkt->gkt_valve3));
	valve_init(&(my_gkt->gkt_valve4));
	valve_init(&(my_gkt->gkt_valve5));
	valve_init(&(my_gkt->gkt_valve6));
	heating_init(&(my_gkt->gkt_heating));
	colding_init(&(my_gkt->gkt_colding));
	cabin_init(&(my_gkt->gkt_cabin_front));
	cabin_init(&(my_gkt->gkt_cabin_back));
	cabin_init(&(my_gkt->gkt_cabin_colding));
	engine_init(&(my_gkt->gkt_engine));

	my_gkt->gkt_valve1.pin = my_gkt->Pin;
	my_gkt->gkt_valve1.pout = my_gkt->gkt_cabin_front.P;
	my_gkt->gkt_valve1.T = my_gkt->Tin;
	my_gkt->gkt_valve1.d = 1;

	my_gkt->gkt_valve2.pin = 800000;
	my_gkt->gkt_valve2.pout = my_gkt->gkt_cabin_front.P;
	my_gkt->gkt_valve2.T = 200;
	my_gkt->gkt_valve2.d = 1;

	my_gkt->gkt_valve3.pin = my_gkt->gkt_cabin_colding.P;
	my_gkt->gkt_valve3.pout = 10000;
	my_gkt->gkt_valve3.T = my_gkt->gkt_cabin_colding.T;
	my_gkt->gkt_valve3.d = 1;

	my_gkt->gkt_valve4.pin = my_gkt->gkt_cabin_front.P;
	my_gkt->gkt_valve4.pout = 100000;
	my_gkt->gkt_valve4.T = my_gkt->gkt_cabin_front.T;
	my_gkt->gkt_valve4.d = 1;

	my_gkt->gkt_engine.ConP = my_gkt->gkt_cabin_front.P;
	my_gkt->gkt_engine.ConT = my_gkt->gkt_cabin_front.T;
	my_gkt->gkt_engine.P = my_gkt->gkt_cabin_back.P;

	my_gkt->gkt_valve5.pin = 100000;
	my_gkt->gkt_valve5.pout = my_gkt->gkt_cabin_back.P;
	my_gkt->gkt_valve5.T = 300;
	my_gkt->gkt_valve5.d = 1;

	my_gkt->gkt_valve6.pin = my_gkt->gkt_cabin_back.P;
	my_gkt->gkt_valve6.pout = 10000;
	my_gkt->gkt_valve6.T = my_gkt->gkt_cabin_back.T;
	my_gkt->gkt_valve6.d = 2;

	my_gkt->gkt_cabin_front.Tin1 = my_gkt->Tin;
	my_gkt->gkt_cabin_front.Tin2 = 200;
	my_gkt->gkt_cabin_front.wa_out = 30;

	my_gkt->gkt_cabin_back.wa_in1 = 60;
	my_gkt->gkt_cabin_back.wa_in2 = 0;
	my_gkt->gkt_cabin_back.Tin1 = 300;
	my_gkt->gkt_cabin_back.Tin2 = 0;

	my_gkt->gkt_cabin_colding.wa_in1 = 300;
	my_gkt->gkt_cabin_colding.wa_in2 = 0;
	my_gkt->gkt_cabin_colding.Tin1 = my_gkt->Tin;
	my_gkt->gkt_cabin_colding.Tin2 = 0;

	my_gkt->gkt_heating.Tin = my_gkt->Tin;
	my_gkt->gkt_heating.Tout = my_gkt->Tin;

	my_gkt->gkt_colding.Tin = my_gkt->Tin;
	my_gkt->gkt_colding.Pin = my_gkt->Pin;
	my_gkt->gkt_colding.Pout = my_gkt->gkt_cabin_colding.P;
}

struct gktout gkt_go(struct gkt *my_gkt, struct gktin my_gktinput)
{
	double wa_out_front, wa_out_colding;
	struct gktout my_gktout;

	my_gkt->gkt_valve1.valve_u = my_gktinput.gkt_valve1_value;
	my_gkt->gkt_valve2.valve_u = my_gktinput.gkt_valve2_value;
	my_gkt->gkt_valve3.valve_u = my_gktinput.gkt_valve3_value;
	my_gkt->gkt_valve4.valve_u = my_gktinput.gkt_valve4_value;
	my_gkt->gkt_valve5.valve_u = my_gktinput.gkt_valve5_value;
	my_gkt->gkt_valve6.valve_u = my_gktinput.gkt_valve6_value;
	my_gkt->gkt_heating.q = my_gktinput.gkt_heating_value;

	//进口
	{
		my_gkt->gkt_cabin_front.wa_in1 = valve_go(&(my_gkt->gkt_valve1), my_gkt->steptime);
		my_gkt->gkt_cabin_front.wa_in2 = valve_go(&(my_gkt->gkt_valve2), my_gkt->steptime);

		my_gkt->gkt_heating.wa = my_gkt->gkt_cabin_front.wa_in1;
		my_gkt->gkt_cabin_front.Tin1 = heating_go(&(my_gkt->gkt_heating), my_gkt->steptime);

		wa_out_colding = my_gkt->gkt_cabin_front.wa_in2;
		wa_out_colding += valve_go(&(my_gkt->gkt_valve3), my_gkt->steptime);
		my_gkt->gkt_cabin_colding.wa_out = wa_out_colding;
		my_gkt->gkt_valve2.pin = cabin_go(&(my_gkt->gkt_cabin_colding), my_gkt->steptime);
		my_gkt->gkt_valve3.pin = my_gkt->gkt_valve2.pin;
		my_gkt->gkt_cabin_colding.Tin1 = colding_go(&(my_gkt->gkt_colding));
		my_gkt->gkt_cabin_front.Tin2 = my_gkt->gkt_cabin_colding.T;

		wa_out_front = engine_go(&(my_gkt->gkt_engine));
		wa_out_front += valve_go(&(my_gkt->gkt_valve4), my_gkt->steptime);
		my_gkt->gkt_cabin_front.wa_out = wa_out_front;
		
		my_gkt->gkt_valve1.pout = cabin_go(&(my_gkt->gkt_cabin_front), my_gkt->steptime);
		my_gkt->gkt_valve2.pout = my_gkt->gkt_valve1.pout;
		my_gkt->gkt_valve4.pin = my_gkt->gkt_valve1.pout;

		my_gkt->gkt_engine.ConP = my_gkt->gkt_cabin_front.P;
		my_gkt->gkt_engine.ConT = my_gkt->gkt_cabin_front.T;
	}

	//出口
	{
		my_gkt->gkt_cabin_back.wa_in1 = valve_go(&(my_gkt->gkt_valve5), my_gkt->steptime);
		my_gkt->gkt_cabin_back.wa_in2 = engine_go(&(my_gkt->gkt_engine));
		my_gkt->gkt_cabin_back.wa_out = valve_go(&(my_gkt->gkt_valve6), my_gkt->steptime);
		my_gkt->gkt_valve5.pout = cabin_go(&(my_gkt->gkt_cabin_back), my_gkt->steptime);
		my_gkt->gkt_valve6.pin = my_gkt->gkt_valve5.pout;

		my_gkt->gkt_engine.P = my_gkt->gkt_cabin_back.P;
	}

	my_gktout.ConT = my_gkt->gkt_cabin_front.T;
	my_gktout.ConP = my_gkt->gkt_cabin_front.P;
	my_gktout.P = my_gkt->gkt_cabin_back.P;

	my_gkt->gkt_gktout = my_gktout;

	return my_gktout;
}

void gkt_valve1_control_init(struct pid *gkt_valve1_pid)
{
	gkt_valve1_pid->error_last = 0;
	gkt_valve1_pid->error_last_last = 0;
	gkt_valve1_pid->Kd = 0.0000;
	gkt_valve1_pid->Ki = 0.001;
	gkt_valve1_pid->Kp = 0.001;
}

double gkt_valve1_control_go(struct pid* gkt_valve1_pid, double u, double RealValue, double ObjectValue)
{
	double error;
	error = ObjectValue - RealValue;

	u += gkt_valve1_pid->Kd * (error - gkt_valve1_pid->error_last) + gkt_valve1_pid->Ki * error + gkt_valve1_pid->Kd * (gkt_valve1_pid->error_last_last - 2 * gkt_valve1_pid->error_last + error);

	if (u > 1)
		u = 1;
	if (u < 0)
		u = 0;

	return u;
}

void gkt_valve2_control_init(struct pid* gkt_valve2_pid)
{
	gkt_valve2_pid->error_last = 0;
	gkt_valve2_pid->error_last_last = 0;
	gkt_valve2_pid->Kd = 0;
	gkt_valve2_pid->Ki = 0.00001;
	gkt_valve2_pid->Kp = 0.00001;
}

double gkt_valve2_control_go(struct pid* gkt_valve2_pid, double u, double RealValue, double ObjectValue)
{
	double error;
	error = ObjectValue - RealValue;

	u += gkt_valve2_pid->Kd * (error - gkt_valve2_pid->error_last) + gkt_valve2_pid->Ki * error + gkt_valve2_pid->Kd * (gkt_valve2_pid->error_last_last - 2 * gkt_valve2_pid->error_last + error);

	if (u > 1)
		u = 1;
	if (u < 0)
		u = 0;

	return u;
}

void gkt_valve3_control_init(struct pid* gkt_valve3_pid)
{
	gkt_valve3_pid->error_last = 0;
	gkt_valve3_pid->error_last_last = 0;
	gkt_valve3_pid->Kd = 0;
	gkt_valve3_pid->Ki = -0.0001;
	gkt_valve3_pid->Kp = -0.0001;
}

double gkt_valve3_control_go(struct pid* gkt_valve3_pid, double u, double RealValue, double ObjectValue)
{
	double error;
	error = ObjectValue - RealValue;

	u += gkt_valve3_pid->Kd * (error - gkt_valve3_pid->error_last) + gkt_valve3_pid->Ki * error + gkt_valve3_pid->Kd * (gkt_valve3_pid->error_last_last - 2 * gkt_valve3_pid->error_last + error);

	if (u > 1)
		u = 1;
	if (u < 0)
		u = 0;

	return u;
}

void gkt_valve4_control_init(struct pid* gkt_valve4_pid)
{
	gkt_valve4_pid->error_last = 0;
	gkt_valve4_pid->error_last_last = 0;
	gkt_valve4_pid->Kd = 0;
	gkt_valve4_pid->Ki = 0.000001;
	gkt_valve4_pid->Kp = 0.000001;
}

double gkt_valve4_control_go(struct pid* gkt_valve4_pid, double u, double RealValue, double ObjectValue)
{
	double error;
	error = ObjectValue - RealValue;

	u += gkt_valve4_pid->Kd * (error - gkt_valve4_pid->error_last) + gkt_valve4_pid->Ki * error + gkt_valve4_pid->Kd * (gkt_valve4_pid->error_last_last - 2 * gkt_valve4_pid->error_last + error);

	if (u > 1)
		u = 1;
	if (u < 0)
		u = 0;

	return u;
}

void gkt_valve5_control_init(struct pid* gkt_valve5_pid)
{
	gkt_valve5_pid->error_last = 0;
	gkt_valve5_pid->error_last_last = 0;
	gkt_valve5_pid->Kd = 0;
	gkt_valve5_pid->Ki = 0.0001;
	gkt_valve5_pid->Kp = 0.0001;
}

double gkt_valve5_control_go(struct pid* gkt_valve5_pid, double u, double RealValue, double ObjectValue)
{
	double error;
	error = ObjectValue - RealValue;

	u += gkt_valve5_pid->Kd * (error - gkt_valve5_pid->error_last) + gkt_valve5_pid->Ki * error + gkt_valve5_pid->Kd * (gkt_valve5_pid->error_last_last - 2 * gkt_valve5_pid->error_last + error);

	if (u > 1)
		u = 1;
	if (u < 0)
		u = 0;

	return u;
}

void gkt_valve6_control_init(struct pid* gkt_valve6_pid)
{
	gkt_valve6_pid->error_last = 0;
	gkt_valve6_pid->error_last_last = 0;
	gkt_valve6_pid->Kd = 0;
	gkt_valve6_pid->Ki = -0.0001;
	gkt_valve6_pid->Kp = -0.0001;
}

double gkt_valve6_control_go(struct pid* gkt_valve6_pid, double u, double RealValue, double ObjectValue)
{
	double error;
	error = ObjectValue - RealValue;

	u += gkt_valve6_pid->Kd * (error - gkt_valve6_pid->error_last) + gkt_valve6_pid->Ki * error + gkt_valve6_pid->Kd * (gkt_valve6_pid->error_last_last - 2 * gkt_valve6_pid->error_last + error);

	if (u > 1)
		u = 1;
	if (u < 0)
		u = 0;

	return u;
}

void gkt_heating_control_init(struct pid* gkt_heating_pid)
{
	gkt_heating_pid->error_last = 0;
	gkt_heating_pid->error_last_last = 0;
	gkt_heating_pid->Kd = 0;
	gkt_heating_pid->Ki = 1;
	gkt_heating_pid->Kp = 1;
}

double gkt_heating_control_go(struct pid* gkt_heating_pid, double u, double RealValue, double ObjectValue)
{
	double error;
	error = ObjectValue - RealValue;

	u += gkt_heating_pid->Kd * (error - gkt_heating_pid->error_last) + gkt_heating_pid->Ki * error + gkt_heating_pid->Kd * (gkt_heating_pid->error_last_last - 2 * gkt_heating_pid->error_last + error);

	return u;
}

void gkt_control_init(struct gkt_control* my_gkt_control)
{
	gkt_valve1_control_init(&(my_gkt_control->gkt_valve1_pid));
	gkt_valve2_control_init(&(my_gkt_control->gkt_valve2_pid));
	gkt_valve3_control_init(&(my_gkt_control->gkt_valve3_pid));
	gkt_valve4_control_init(&(my_gkt_control->gkt_valve4_pid));
	gkt_valve5_control_init(&(my_gkt_control->gkt_valve5_pid));
	gkt_valve6_control_init(&(my_gkt_control->gkt_valve6_pid));
	gkt_heating_control_init(&(my_gkt_control->gkt_heating_pid));
}

struct gktin gkt_control_go(struct gkt_control* my_gkt_control, struct gktin my_gktin, struct gkt_control_object RealValue, struct gkt_control_object ObjectValue)
{
	struct gktin u;

	u.gkt_valve1_value = gkt_valve1_control_go(&(my_gkt_control->gkt_valve1_pid), my_gktin.gkt_valve1_value, RealValue.ConP, ObjectValue.ConP);
	u.gkt_valve2_value = gkt_valve2_control_go(&(my_gkt_control->gkt_valve2_pid), my_gktin.gkt_valve2_value, RealValue.ConT, ObjectValue.ConT);
	u.gkt_valve3_value = gkt_valve3_control_go(&(my_gkt_control->gkt_valve3_pid), my_gktin.gkt_valve3_value, RealValue.colding_P, ObjectValue.colding_P);
	u.gkt_valve4_value = gkt_valve4_control_go(&(my_gkt_control->gkt_valve4_pid), my_gktin.gkt_valve4_value, RealValue.ConP, ObjectValue.ConP);
	u.gkt_valve5_value = gkt_valve5_control_go(&(my_gkt_control->gkt_valve5_pid), my_gktin.gkt_valve5_value, RealValue.P, ObjectValue.P);
	u.gkt_valve6_value = gkt_valve6_control_go(&(my_gkt_control->gkt_valve6_pid), my_gktin.gkt_valve6_value, RealValue.P, ObjectValue.P);
	u.gkt_heating_value = gkt_heating_control_go(&(my_gkt_control->gkt_heating_pid), my_gktin.gkt_heating_value, RealValue.heating_T, ObjectValue.heating_T);

	return u;
}

void pid_go()
{
	int i;

	struct gkt my_gkt;
	struct gktin my_gktin;
	struct gktout my_gktout;
	struct pid gkt_valve1_pid;
	struct pid gkt_valve2_pid;
	struct pid gkt_valve3_pid;
	struct pid gkt_valve4_pid;
	struct pid gkt_valve5_pid;
	struct pid gkt_valve6_pid;
	struct pid gkt_heating_pid;

	FILE* fp;
	errno_t err = fopen_s(&fp, "output_pid.txt", "w");
	if (fp == NULL) {
		printf("Error! Could not open file\n");
	}

	my_gktin.gkt_valve1_value = 0.5;
	my_gktin.gkt_valve2_value = 0.5;
	my_gktin.gkt_valve3_value = 0.5;
	my_gktin.gkt_valve4_value = 0.5;
	my_gktin.gkt_valve5_value = 0.5;
	my_gktin.gkt_valve6_value = 0.5;
	my_gktin.gkt_heating_value = 10000;

	gkt_init(&my_gkt);
	gkt_valve1_control_init(&gkt_valve1_pid);
	gkt_valve2_control_init(&gkt_valve2_pid);
	gkt_valve3_control_init(&gkt_valve3_pid);
	gkt_valve4_control_init(&gkt_valve4_pid);
	gkt_valve5_control_init(&gkt_valve5_pid);
	gkt_valve6_control_init(&gkt_valve6_pid);
	gkt_heating_control_init(&gkt_heating_pid);

	for (i = 0; i < 1000; i++) {
		my_gktout = gkt_go(&my_gkt, my_gktin);
		my_gktin.gkt_valve1_value = gkt_valve1_control_go(&gkt_valve1_pid, my_gktin.gkt_valve1_value, my_gkt.gkt_cabin_front.T, 250);
		my_gktin.gkt_valve2_value = gkt_valve2_control_go(&gkt_valve2_pid, my_gktin.gkt_valve2_value, my_gkt.gkt_cabin_front.P, 200000);
		my_gktin.gkt_valve3_value = gkt_valve3_control_go(&gkt_valve3_pid, my_gktin.gkt_valve3_value, my_gkt.gkt_cabin_colding.P, 500000);
		my_gktin.gkt_valve4_value = gkt_valve4_control_go(&gkt_valve4_pid, my_gktin.gkt_valve4_value, my_gkt.gkt_cabin_front.P, 200000);
		my_gktin.gkt_valve5_value = gkt_valve5_control_go(&gkt_valve5_pid, my_gktin.gkt_valve5_value, my_gkt.gkt_cabin_back.P, 20000);
		my_gktin.gkt_valve6_value = gkt_valve6_control_go(&gkt_valve6_pid, my_gktin.gkt_valve6_value, my_gkt.gkt_cabin_back.P, 20000);
		my_gktin.gkt_heating_value = gkt_heating_control_go(&gkt_heating_pid, my_gktin.gkt_heating_value, my_gkt.gkt_heating.Tout, 350);
		printf("ConT: %.6f ConP: %.6f P: %.6f\n", my_gktout.ConT, my_gktout.ConP, my_gktout.P);
	}
	for (i = 0; i < 10; i++) {
		my_gktout = gkt_go(&my_gkt, my_gktin);
		my_gktin.gkt_valve1_value = gkt_valve1_control_go(&gkt_valve1_pid, my_gktin.gkt_valve1_value, my_gkt.gkt_cabin_front.T, 250);
		my_gktin.gkt_valve2_value = gkt_valve2_control_go(&gkt_valve2_pid, my_gktin.gkt_valve2_value, my_gkt.gkt_cabin_front.P, 200000);
		my_gktin.gkt_valve3_value = gkt_valve3_control_go(&gkt_valve3_pid, my_gktin.gkt_valve3_value, my_gkt.gkt_cabin_colding.P, 500000);
		my_gktin.gkt_valve4_value = gkt_valve4_control_go(&gkt_valve4_pid, my_gktin.gkt_valve4_value, my_gkt.gkt_cabin_front.P, 200000);
		my_gktin.gkt_valve5_value = gkt_valve5_control_go(&gkt_valve5_pid, my_gktin.gkt_valve5_value, my_gkt.gkt_cabin_back.P, 20000);
		my_gktin.gkt_valve6_value = gkt_valve6_control_go(&gkt_valve6_pid, my_gktin.gkt_valve6_value, my_gkt.gkt_cabin_back.P, 20000);
		my_gktin.gkt_heating_value = gkt_heating_control_go(&gkt_heating_pid, my_gktin.gkt_heating_value, my_gkt.gkt_heating.Tout, 350);
		printf("ConT: %.6f ConP: %.6f P: %.6f\n", my_gktout.ConT, my_gktout.ConP, my_gktout.P);
		fprintf(fp, "%.6f %.6f %.6f\n", my_gktout.ConT, my_gktout.ConP, my_gktout.P);
	}

	my_gkt.gkt_engine.Acr = 1;
	printf("\n********************Acr Change: Acr = 1***********************\n\n");
	for (i = 0; i < 100; i++) {
		my_gktout = gkt_go(&my_gkt, my_gktin);
		my_gktin.gkt_valve1_value = gkt_valve1_control_go(&gkt_valve1_pid, my_gktin.gkt_valve1_value, my_gkt.gkt_cabin_front.T, 250);
		my_gktin.gkt_valve2_value = gkt_valve2_control_go(&gkt_valve2_pid, my_gktin.gkt_valve2_value, my_gkt.gkt_cabin_front.P, 200000);
		my_gktin.gkt_valve3_value = gkt_valve3_control_go(&gkt_valve3_pid, my_gktin.gkt_valve3_value, my_gkt.gkt_cabin_colding.P, 500000);
		my_gktin.gkt_valve4_value = gkt_valve4_control_go(&gkt_valve4_pid, my_gktin.gkt_valve4_value, my_gkt.gkt_cabin_front.P, 200000);
		my_gktin.gkt_valve5_value = gkt_valve5_control_go(&gkt_valve5_pid, my_gktin.gkt_valve5_value, my_gkt.gkt_cabin_back.P, 20000);
		my_gktin.gkt_valve6_value = gkt_valve6_control_go(&gkt_valve6_pid, my_gktin.gkt_valve6_value, my_gkt.gkt_cabin_back.P, 20000);
		my_gktin.gkt_heating_value = gkt_heating_control_go(&gkt_heating_pid, my_gktin.gkt_heating_value, my_gkt.gkt_heating.Tout, 350);
		printf("ConT: %.6f ConP: %.6f P: %.6f\n", my_gktout.ConT, my_gktout.ConP, my_gktout.P);
		fprintf(fp, "%.6f %.6f %.6f\n", my_gktout.ConT, my_gktout.ConP, my_gktout.P);
		if (i == 998)
			i = 998;
	}
	fclose(fp);
	getchar();
}
