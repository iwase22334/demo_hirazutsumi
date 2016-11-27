//----------------------------------------------------------
// Hajime Iwase
// 2015/04/01/
// v1.00
// Class of fixing the processing period
//----------------------------------------------------------
#ifndef TIMER_H
#define TIMER_H
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <sys/time.h>
#include <unistd.h>
//----------------------------------------------------------
// class
//----------------------------------------------------------
class Timer{
protected:
	int lap;
	double freq;
	double period;
	struct timeval start_t;
	struct timeval stop_t;

public:
	// Constructor
	Timer();
	Timer(double);

	Timer& operator=(const Timer &);

	// Setter
	void set_freq(const double);
	void set_period(const double);
	// Getter
	int get_lap() const;

	double get_freq() const;
	double get_period() const;
	double get_start() const;
	double get_stop() const;
	double get_interval() const;

	// Start timer
	double start();
	// Stop timer
	double stop();
	// Stop program execution
	double sleep();
	// Reset timer
	void reset();

private:
	double convert_time(const struct timeval & t) const {
		return ((double)(t.tv_sec) + (double)(t.tv_usec) * 0.001 * 0.001);
	};

};
#endif