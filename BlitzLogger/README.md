#=--------------------------------------------------------------------=
#=  .______       _______     ___       _______  .___  ___.  _______  =
#=  |   _  \     |   ____|   /   \     |       \ |   \/   | |   ____| =
#=  |  |_)  |    |  |__     /  ^  \    |  .--.  ||  \  /  | |  |__    =
#=  |      /     |   __|   /  /_\  \   |  |  |  ||  |\/|  | |   __|   =
#=  |  |\  \----.|  |____ /  _____  \  |  '--'  ||  |  |  | |  |____  =
#=  | _| `._____||_______/__/     \__\ |_______/ |__|  |__| |_______| =
#=      	                                                          =
#=--------------------------------------------------------------------=
BlitzLogger creates a textfile that must be opened in NotePad++ and 
can be used to debug and take logs of what our robot is doing

There are 5 LogLevels

Error    - 	  0
Warning  - 	  1
Info     - 	  2
Debug    -    3
Trace    -    4

When you start make the constructor for the BlitzLogger item you must
include the logLevel

'''C++
	BlitzLogger(/* Put LogLevel Here */)
'''
	