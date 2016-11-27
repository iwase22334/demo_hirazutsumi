//----------------------------------------------------------
// Hajime Iwase
// 2015/04/01/
// v1.00
// Class of fixing the processing period
//----------------------------------------------------------
#include"timer.h"
const int INT_MAX = 32767;
//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
Timer::Timer(){
	freq = 0;
	period = 0;
	lap = 0;
};


Timer::Timer(double i_freq){
	period = (1 / i_freq);
	freq = i_freq;
	lap = 0;
};


Timer& Timer::operator=(const Timer & t){
	lap = t.lap;
	freq = t.freq;
	period = t.period ;
	start_t = t.start_t;
	stop_t = t.stop_t;
	return *this;
};
//----------------------------------------------------------
// Setter
//----------------------------------------------------------
void Timer::set_freq(const double i_freq){
	freq = i_freq;
	period = 1.0 / (double)i_freq;
};

void Timer::set_period(const double i_period){
	freq = (int)(1.0 / i_period);
	period = i_period;
};

//----------------------------------------------------------
// Getter
//----------------------------------------------------------
double Timer::get_freq() const { return freq; };
double Timer::get_period() const { return period; };
int Timer::get_lap() const { return lap; };
double Timer::get_start() const { return convert_time(start_t); };
double Timer::get_stop() const { return convert_time(stop_t); };

double Timer::get_interval() const{ 
	struct timeval dt_t;
	dt_t.tv_sec = stop_t.tv_sec - start_t.tv_sec;
	dt_t.tv_usec = stop_t.tv_usec - start_t.tv_usec;
	return convert_time(dt_t);
};

//----------------------------------------------------------
// Start timer
//----------------------------------------------------------
double Timer::start(){
	gettimeofday(&start_t, NULL);
	return convert_time(start_t);
};

//----------------------------------------------------------
// Stop timer
//----------------------------------------------------------
double Timer::stop(){
	gettimeofday(&stop_t, NULL);
	return convert_time(stop_t);
};

//----------------------------------------------------------
// Stop Program execution
//----------------------------------------------------------
double Timer::sleep(){
	double dt = get_interval();
	double us = 0;
	if(dt < period){
		us = (period - dt) * 1000.0 * 1000.0;
		++ lap;
		lap %= INT_MAX;
		usleep(us);
	}
	return us;
};

