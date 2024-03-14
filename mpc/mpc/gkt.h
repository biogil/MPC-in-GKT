#ifndef gkt_h
#define gkt_h

//物理单元
struct valve{
	double d; //直径
	double valve_u; //调节阀开度
	double Wa; //流量
	double pin; //输入压力
	double pout; //输出压力
	double T; //输入温度
	double phi; //流量系数
};

void valve_init(struct valve *my_valve);
double valve_go(struct valve *my_valve, double steptime);

struct cabin{
	double wa_in1; //输入流量1
	double wa_in2; //输入流量2
	double wa_out; //输出流量
	double Tin1; //输入温度1
	double Tin2; //输入温度2
	double T; //容腔温度
	double P; //容腔压力
	double Cin1; //输入流速1
	double Cin2; //输入流速2
	double Cout; //输出流速
	double V; //容腔体积
};

void cabin_init(struct cabin *my_cabin);
double cabin_go(struct cabin *my_cabin, double steptime);

struct heating {
	double wa;
	double c;
	double C;
	double Tin;
	double Tout;
	double q;
};

void heating_init(struct heating* my_heating);
double heating_go(struct heating *my_heating, double steptime);

struct colding {
	double eta;
	double Tin;
	double Tout;
	double Pin;
	double Pout;
};

void colding_init(struct colding* my_colding);
double colding_go(struct colding* my_colding);

struct engine {
	double Acr;
	double ConT;
	double ConP;
	double P;
	double Wa;
};

void engine_init(struct engine* my_engine);
double engine_go(struct engine* my_engine);

struct gktin{
	double gkt_valve1_value;
	double gkt_valve2_value;
	double gkt_valve3_value;
	double gkt_valve4_value;
	double gkt_valve5_value;
	double gkt_valve6_value;
	double gkt_heating_value;
};

struct gktout{
	double ConT;
	double ConP;
	double P;
};

struct gkt{
	struct gktin my_gktin;
	struct gktout gkt_gktout;
	struct valve gkt_valve1;
	struct valve gkt_valve2;
	struct valve gkt_valve3;
	struct valve gkt_valve5;
	struct valve gkt_valve4;
	struct valve gkt_valve6;
	struct heating gkt_heating;
	struct colding gkt_colding;

	struct cabin gkt_cabin_front;
	struct cabin gkt_cabin_back;
	struct cabin gkt_cabin_colding;

	struct engine gkt_engine;
	
	double steptime;
	double Pin;
	double Tin;
	double Pout;
};

void gkt_init(struct gkt *my_gkt);
struct gktout gkt_go(struct gkt *my_gkt, struct gktin my_gktinput);

//PID控制单元；
struct pid {
	double error_last;
	double error_last_last;
	double Kp;
	double Ki;
	double Kd;
};

void gkt_valve1_control_init(struct pid* gkt_valve1_pid);
double gkt_valve1_control_go(struct pid* gkt_valve1_pid, double u, double RealValue, double ObjectValue);

void gkt_valve2_control_init(struct pid* gkt_valve2_pid);
double gkt_valve2_control_go(struct pid* gkt_valve2_pid, double u, double RealValue, double ObjectValue);

void gkt_valve3_control_init(struct pid* gkt_valve3_pid);
double gkt_valve3_control_go(struct pid* gkt_valve3_pid, double u, double RealValue, double ObjectValue);

void gkt_valve4_control_init(struct pid* gkt_valve4_pid);
double gkt_valve4_control_go(struct pid* gkt_valve4_pid, double u, double RealValue, double ObjectValue);

void gkt_valve5_control_init(struct pid* gkt_valve5_pid);
double gkt_valve5_control_go(struct pid* gkt_valve5_pid, double u, double RealValue, double ObjectValue);

void gkt_valve6_control_init(struct pid* gkt_valve6_pid);
double gkt_valve6_control_go(struct pid* gkt_valve6_pid, double u, double RealValue, double ObjectValue);

void gkt_heating_control_init(struct pid* gkt_heating_pid);
double gkt_heating_control_go(struct pid* gkt_heating_pid, double u, double RealValue, double ObjectValue);

struct gkt_control {
	struct pid gkt_valve1_pid;
	struct pid gkt_valve2_pid;
	struct pid gkt_valve3_pid;
	struct pid gkt_valve4_pid;
	struct pid gkt_valve5_pid;
	struct pid gkt_valve6_pid;
	struct pid gkt_heating_pid;
};

struct gkt_control_object {
	double ConT;
	double ConP;
	double P;
	double heating_T;
	double colding_P;
};

void gkt_control_init(struct gkt_control* my_gkt_control);
struct gktin gkt_control_go(struct gkt_control* my_gkt_control, struct gktin my_gktin, struct gkt_control_object RealValue, struct gkt_control_object ObjectValue);

void pid_go();


#endif

