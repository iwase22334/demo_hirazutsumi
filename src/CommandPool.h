//------------------------------------------------------------
// @author: Iwase Hajime
// @date : 2016.03.11
// @Description : This code is Pool of Command.
//				Command Pool retains the instance of Commands.
//				Commands are passed to RobotHandler as Pointer.
//------------------------------------------------------------
#include <list>
#include "command.h"
//----------------------------------------------------------
// class
//----------------------------------------------------------
struct CommandPool{
public:
	list<Command> basic;
	list<OPLCommand> opl;
	list<ThroughHoleCommand> th;
public:
	void clear(){
		basic.clear();
		opl.clear();
		th.clear();
	};
};